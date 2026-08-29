#!/usr/bin/env python3
"""Global localization manager (Plan A: Scan Context + GICP).

Fully autonomous relocalization of the current FAST-LIO session against a
saved 3D map. No manual initial pose.

Pipeline
  1. Accumulate a short window of /cloud_registered (camera_init frame).
  2. Describe the window with Scan Context; retrieve top-k keyframe
     candidates from the map database (db.npz built by build_map_db).
  3. For every candidate: crop a local submap from the map PCD around the
     candidate pose and register the window against it (small_gicp, with an
     open3d fallback), using the candidate pose as the initial guess.
  4. Uniqueness check: the best candidate must clearly beat the runner-up.
     If ambiguous, stay SEARCHING — never guess (硬约束: 定位不唯一就停止).
  5. VERIFYING: require N consecutive consistent alignments before
     publishing. Then LOCALIZED: publish the dynamic TF map_level→camera_init
     and keep tracking at ~1 Hz against a local submap, degrading to
     DEGRADED/LOST when match quality drops. LOST restarts a full search.

State is published to runtime/localization_state.json for the web UI and the
motion bridge hard gate (only LOCALIZED allows GO2 motion).

TF chain: keyframe poses are stored in the map frame (the mapping session's
camera_init). The published TF chains the fixed alignment transform
(localization_alignment.json, map_level←map) with the estimated
map←camera_init, so checkpoints and the web cloud stay in map_level.
"""
import json
import math
import os
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster

RUNTIME = Path(os.environ.get('PATROL_RUNTIME', '/project/runtime'))
DATA = Path(os.environ.get('PATROL_DATA', '/project/patrol_data'))

WINDOW_SEC = 4.0
WINDOW_VOXEL = 0.20
SUBMAP_RADIUS = 15.0
TRACK_RADIUS = 25.0
SEARCH_PERIOD = 1.0

RMSE_MAX = float(os.environ.get('LOC_RMSE_MAX', '0.25'))
OVERLAP_MIN = float(os.environ.get('LOC_OVERLAP_MIN', '0.50'))
UNIQUE_MARGIN = float(os.environ.get('LOC_UNIQUE_MARGIN', '0.05'))
VERIFY_COUNT = 5
VERIFY_POS_TOL = 0.15
VERIFY_YAW_TOL = math.radians(5.0)
CORRECT_MAX = 0.30
CORRECT_YAW_MAX = math.radians(10.0)
LOST_CORRECT_MAX = 1.0


def quat_yaw(q):
    return math.atan2(2.0 * (q.z * q.w + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rpy_from_matrix(r):
    pitch = math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0]))
    if abs(pitch) < math.pi / 2 - 1e-6:
        roll = math.atan2(r[2, 1], r[2, 2])
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:
        roll = math.atan2(-r[1, 2], r[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def transform_matrix(x, y, z, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, z],
                     [0, 0, 0, 1]], dtype=np.float64)


def voxel_downsample(points, voxel):
    if not len(points):
        return points
    keys = np.floor(points[:, :3] / voxel).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(index)]


class Registration:
    """small_gicp backend with an open3d fallback. align() returns
    (T_target_source 4x4, rmse, fitness) or raises RuntimeError."""

    def __init__(self, node):
        self.node = node
        self.backend = None
        try:
            import small_gicp  # noqa: F401
            self.backend = 'small_gicp'
        except ImportError:
            try:
                import open3d  # noqa: F401
                self.backend = 'open3d'
            except ImportError:
                pass
        if self.backend is None:
            raise RuntimeError('未找到配准后端：请在镜像中安装 small_gicp 或 open3d')
        node.get_logger().info(f'配准后端: {self.backend}')

    def align(self, target, source, init_t):
        target = np.ascontiguousarray(target[:, :3], dtype=np.float64)
        source = np.ascontiguousarray(source[:, :3], dtype=np.float64)
        if self.backend == 'small_gicp':
            import small_gicp
            result = small_gicp.align(
                target, source,
                init_T_target_source=init_t,
                downsample_resolution=WINDOW_VOXEL,
                max_correspondence_distance=1.0,
                num_threads=2)
            t = np.asarray(result.T_target_source, dtype=np.float64)
            rmse = float(result.rmse)
            fitness = self._overlap(target, source @ t[:3, :3].T + t[:3, 3])
            return t, rmse, fitness
        import open3d as o3d
        to3d = lambda p: o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(np.ascontiguousarray(p[:, :3])))
        t_cloud = to3d(target)
        s_cloud = to3d(source)
        s_cloud.transform(init_t.tolist())
        reg = o3d.pipelines.registration.registration_icp(
            s_cloud, t_cloud, 1.0,
            np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        t = np.asarray(reg.transformation, dtype=np.float64) @ init_t
        fitness = float(reg.fitness)
        rmse = float(reg.inlier_rmse) if reg.fitness > 0 else 1e3
        return t, rmse, fitness

    @staticmethod
    def _overlap(target, transformed, cell=0.3):
        """Fraction of transformed source points falling into an occupied
        target voxel cell — cheap overlap metric without a KD-tree."""
        if not len(target) or not len(transformed):
            return 0.0
        t_keys = np.floor(target[:, :3] / cell).astype(np.int64)
        s_keys = np.floor(transformed[:, :3] / cell).astype(np.int64)
        t_set = set(map(tuple, np.unique(t_keys, axis=0)))
        s_set = set(map(tuple, np.unique(s_keys, axis=0)))
        if not s_set:
            return 0.0
        return len(t_set & s_set) / len(s_set)


class LocalizationManager(Node):
    def __init__(self):
        super().__init__('localization_manager')
        self.map_name = os.environ.get('LOC_MAP_NAME', '')
        self.state = 'UNINITIALIZED'
        self.map_id = None
        self.dbs = {}          # map_name -> {'poses','descriptors','sc_shape'}
        self.pcd = {}          # map_name -> points (N,3) map frame
        self.registration = Registration(self)
        self.window = []       # (t, points Nx3 camera_init)
        self.last_process = 0.0
        self.t_map_cam = None  # 4x4 map←camera_init
        self.map_id_est = None
        self.verify_results = []
        self.degraded_count = 0
        self.last_correction = None
        self.alignment = self.load_alignment()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, '/cloud_registered',
                                 self.on_cloud, qos)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, qos)
        self.odo_z = 0.0
        self.candidate = None
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0, self.tick)
        self.write_state()
        self.get_logger().info('全局定位管理器就绪（SEARCHING 待点云）')

    # ---------- data loading ----------
    def load_alignment(self):
        try:
            a = json.loads((RUNTIME / 'localization_alignment.json').read_text(
                encoding='utf-8'))
        except (OSError, ValueError):
            a = {}
        yaw = float(a.get('yaw', 0))
        c, s = math.cos(yaw), math.sin(yaw)
        t = np.array([float(a.get('x', 0)), float(a.get('y', 0)),
                      float(a.get('z', 0))])
        return {'rot': np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]), 't': t}

    def load_maps(self):
        for db in sorted(DATA.glob('maps/*/db.npz')):
            name = db.parent.name
            if name in self.dbs:
                continue
            try:
                data = np.load(db)
                pcd = next(db.parent.glob('*.pcd'))
                pts = self.load_pcd(pcd)
                if pts is None or len(data['poses']) < 3:
                    continue
                self.dbs[name] = data
                self.pcd[name] = pts
                self.get_logger().info(
                    f'载入地图数据库: {name} 关键帧={len(data["poses"])}')
            except (OSError, ValueError, KeyError) as exc:
                self.get_logger().warning(f'跳过地图 {name}: {exc}')
        if not self.dbs:
            self.get_logger().error('没有任何可用地图数据库（需要先建图生成 db.npz）')

    @staticmethod
    def load_pcd(path):
        """Binary/ascii PCD reader for x/y/z float32 clouds."""
        data = Path(path).read_bytes()
        try:
            header_end = data.index(b'DATA ')
        except ValueError:
            return None
        kind_line = data[header_end:header_end + 24].split(b'\n')[0]
        kind = kind_line.split()[1].decode()
        body_start = data.index(b'\n', header_end) + 1
        fields = []
        count = 0
        for line in data[:header_end].decode('ascii', errors='replace').splitlines():
            parts = line.split()
            if not parts:
                continue
            if parts[0] == 'FIELDS':
                fields = parts[1:]
            elif parts[0] == 'POINTS':
                count = int(parts[1])
        float_cols = sum(1 for f in fields if f in ('x', 'y', 'z', 'intensity'))
        if count <= 0 or float_cols < 3:
            return None
        if kind == 'ascii':
            rows = np.loadtxt(data[body_start:].splitlines()[:count],
                              usecols=(0, 1, 2))
            return rows.reshape(-1, 3)
        arr = np.frombuffer(data[body_start:body_start + count * 4 * float_cols],
                            dtype=np.float32)
        return arr.reshape(-1, float_cols)[:, :3].copy()

    # ---------- callbacks ----------
    def on_odom(self, msg):
        self.odo_z = float(msg.pose.pose.position.z)

    def on_cloud(self, msg):
        try:
            points = point_cloud2.read_points_numpy(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception:
            return
        if len(points) < 100:
            return
        now = time.monotonic()
        self.window.append((now, np.ascontiguousarray(points[:, :3])))
        while self.window and now - self.window[0][0] > WINDOW_SEC + 2.0:
            self.window.pop(0)

    # ---------- window helpers ----------
    def window_points(self):
        if not self.window:
            return None
        now = time.monotonic()
        recent = [p for t, p in self.window if now - t <= WINDOW_SEC]
        if not recent:
            return None
        pts = np.vstack(recent)
        if len(pts) < 2000:
            return None
        return voxel_downsample(pts, WINDOW_VOXEL)

    # ---------- main cycle ----------
    def tick(self):
        window = self.window_points()
        if window is None:
            if self.state in ('SEARCHING', 'UNINITIALIZED'):
                self.set_state('SEARCHING', '等待当前点云积累')
            return
        now = time.monotonic()
        if now - self.last_process < SEARCH_PERIOD:
            return
        self.last_process = now

        if self.state in ('UNINITIALIZED', 'SEARCHING', 'LOST', 'AMBIGUOUS'):
            self.do_search(window)
        elif self.state == 'VERIFYING':
            self.do_verify(window)
        elif self.state in ('LOCALIZED', 'DEGRADED'):
            self.do_track(window)
        self.publish_tf()
        self.write_state()

    # ---------- search / verify / track ----------
    def do_search(self, window):
        if not self.dbs:
            self.load_maps()
            if not self.dbs:
                self.set_state('SEARCHING', '无可用地图数据库')
                return
        results = self.match_all(window)
        if not results:
            self.set_state('SEARCHING', '所有候选配准失败')
            return
        best, second = results[0], (results[1] if len(results) > 1 else None)
        # Uniqueness: a runner-up at a DIFFERENT place (>2 m away) with a
        # close score means the environment is ambiguous — refuse to guess.
        # Runners-up near the best position are the same place (nearby
        # keyframes) and are harmless.
        ambiguous = (second is not None and
                     second['fitness'] > best['fitness'] - UNIQUE_MARGIN and
                     math.hypot(second['x'] - best['x'],
                                second['y'] - best['y']) > 2.0)
        if ambiguous:
            self.set_state('SEARCHING',
                           f"候选不唯一(次选 {second['map']}"
                           f"@({second['x']:.1f},{second['y']:.1f}) "
                           f"fitness={second['fitness']:.2f})，拒绝猜测")
            return
        self.candidate = best
        self.verify_results = []
        self.set_state('VERIFYING',
                       f"候选 {best['map']}@({best['x']:.1f},{best['y']:.1f}) "
                       f"fitness={best['fitness']:.2f} rmse={best['rmse']:.2f}")

    def do_verify(self, window):
        best = self.candidate
        result = self.align_one(best['map'], window, best['t'],
                                submap_radius=SUBMAP_RADIUS)
        if result is None:
            self.verify_results = []
            self.set_state('SEARCHING', '验证期配准失败，重新检索')
            return
        t, rmse, fitness = result
        pos = t[:3, 3]
        consistent = True
        for prev_t, prev_rmse, prev_fit in self.verify_results:
            if (np.linalg.norm(pos - prev_t[:3, 3]) > VERIFY_POS_TOL or
                    abs(yaw_of(t) - yaw_of(prev_t)) > VERIFY_YAW_TOL):
                consistent = False
                break
        if not consistent:
            self.verify_results = [(t, rmse, fitness)]
            self.set_state('VERIFYING', '验证帧不一致，重新计数')
            return
        self.verify_results.append((t, rmse, fitness))
        if len(self.verify_results) >= VERIFY_COUNT:
            self.t_map_cam = t
            self.map_id_est = best['map']
            self.degraded_count = 0
            self.last_correction = None
            self._last_rmse = rmse
            self.set_state('LOCALIZED',
                           f"定位成功 map={best['map']} fitness={fitness:.2f}")
        else:
            self.set_state('VERIFYING',
                           f'验证 {len(self.verify_results)}/{VERIFY_COUNT}')

    def do_track(self, window):
        result = self.align_one(self.map_id_est, window, self.t_map_cam,
                                submap_radius=TRACK_RADIUS)
        if result is None:
            self.degraded_count += 3
        else:
            t, rmse, fitness = result
            jump = np.linalg.norm(t[:3, 3] - self.t_map_cam[:3, 3])
            jump_yaw = abs(yaw_of(t) - yaw_of(self.t_map_cam))
            if jump > LOST_CORRECT_MAX or jump_yaw > math.radians(20):
                self.set_state('LOST', f'修正量异常({jump:.2f}m)，重新全局检索')
                self.degraded_count = 0
                return
            self.t_map_cam = t
            self._last_rmse = rmse
            if (rmse > RMSE_MAX or fitness < OVERLAP_MIN or
                    jump > CORRECT_MAX or jump_yaw > CORRECT_YAW_MAX):
                self.degraded_count += 1
            else:
                self.degraded_count = 0
        if self.degraded_count >= 6:
            self.set_state('LOST', '连续匹配失败，重新全局检索')
            self.degraded_count = 0
        elif self.degraded_count >= 3:
            self.set_state('DEGRADED', '匹配质量下降')

    # ---------- registration helpers ----------
    def match_all(self, window):
        """Full search across all loaded maps. Returns scored candidates."""
        from patrol_global_localization.scan_context import (
            make_descriptor_height, search)
        sensor_z = self.odo_z
        query = make_descriptor_height(window, sensor_z=sensor_z)
        candidates = []
        for map_name, db in self.dbs.items():
            entries = [{'sc': d.reshape(tuple(db['sc_shape'])), 'index': i}
                       for i, d in enumerate(db['descriptors'])]
            try:
                top = search(entries, query, topk=5)
            except Exception as exc:
                self.get_logger().warning(f'{map_name} 检索失败: {exc}')
                continue
            for entry, dist, shift in top:
                pose = db['poses'][entry['index']]
                candidates.append({'map': map_name, 'index': entry['index'],
                                   'x': float(pose[0]), 'y': float(pose[1]),
                                   'z': float(pose[2]), 'yaw': float(pose[3]),
                                   'sc_dist': dist})
        scored = []
        for cand in candidates[:8]:
            init_t = transform_matrix(cand['x'], cand['y'], cand['z'],
                                      cand['yaw'])
            result = self.align_one(cand['map'], window, init_t,
                                    submap_radius=SUBMAP_RADIUS)
            if result is None:
                continue
            t, rmse, fitness = result
            scored.append({**cand, 't': t, 'rmse': rmse, 'fitness': fitness})
        scored.sort(key=lambda c: (-c['fitness'], c['rmse']))
        return scored

    def align_one(self, map_name, window, init_t, submap_radius):
        pts = self.pcd.get(map_name)
        if pts is None:
            return None
        center = (init_t @ np.array([0, 0, 0, 1.0]))[:3]
        d = np.hypot(pts[:, 0] - center[0], pts[:, 1] - center[1])
        submap = pts[d <= submap_radius]
        if len(submap) < 1000:
            return None
        try:
            return self.registration.align(submap, window, np.asarray(init_t,
                                                                       dtype=np.float64))
        except Exception as exc:
            self.get_logger().warning(f'配准异常: {exc}')
            return None

    # ---------- outputs ----------
    def publish_tf(self):
        if self.t_map_cam is None or self.state not in ('LOCALIZED', 'DEGRADED'):
            return
        a_rot, a_t = self.alignment['rot'], self.alignment['t']
        full = a_rot @ self.t_map_cam[:3, :3]
        trans = a_rot @ self.t_map_cam[:3, 3] + a_t
        roll, pitch, yaw = rpy_from_matrix(full)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map_level'
        msg.child_frame_id = 'camera_init'
        msg.transform.translation.x, msg.transform.translation.y, \
            msg.transform.translation.z = trans
        qx, qy, qz, qw = quat_from_rpy(roll, pitch, yaw)
        msg.transform.rotation.x, msg.transform.rotation.y = qx, qy
        msg.transform.rotation.z, msg.transform.rotation.w = qz, qw
        self.tf_broadcaster.sendTransform(msg)

    def set_state(self, state, message):
        if state != self.state:
            self.get_logger().info(f'定位状态: {self.state} -> {state} ({message})')
            self.state = state
        self.state_message = message

    def write_state(self):
        t = self.t_map_cam
        payload = {
            'state': self.state,
            'map_id': self.map_id_est,
            'message': getattr(self, 'state_message', ''),
            'position': [round(float(v), 3) for v in t[:3, 3]] if t is not None else None,
            'yaw': round(float(yaw_of(t)), 3) if t is not None else None,
            'rmse': getattr(self, '_last_rmse', None),
            'updated_at': time.time(),
        }
        if t is not None and self.state in ('LOCALIZED', 'DEGRADED'):
            payload['ok_for_navigation'] = True
        else:
            payload['ok_for_navigation'] = False
        tmp = Path(str(RUNTIME / 'localization_state.json') + '.tmp')
        tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=1),
                       encoding='utf-8')
        os.replace(tmp, RUNTIME / 'localization_state.json')


def yaw_of(t):
    return math.atan2(t[1, 0], t[0, 0])


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy + sr * sp * cy, cr * cp * cy + sr * sp * sy)


def main():
    rclpy.init()
    node = LocalizationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
