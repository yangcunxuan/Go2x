#!/usr/bin/env python3
"""Global localization manager (Plan A: Scan Context + two-stage GICP).

Fully autonomous relocalization of the current FAST-LIO session against a
saved map package. No manual initial pose; no fallback transforms.

Pipeline per cycle:
  1. Accumulate a short window of /cloud_registered (camera_init frame).
  2. Describe the window in the CURRENT SENSOR frame (P0 audit #6: the
     window is camera_init-world and must be centered by the live odometry
     pose before the descriptor).
  3. Scan Context retrieval over every map database (ring-key coarse
     ranking, then exact sector-shift matching). Candidates from all maps
     are merged and globally sorted by descriptor distance.
  4. For each of the top candidates: initial transform
        T(mapLevel<-cameraInit) =
            T(mapLevel<-sensor_keyframe)
            x Rz(yaw_shift)
            x inverse(T(cameraInit<-sensor_current))
     then two-stage small_gicp (coarse 0.5 m / 2.5 m, fine 0.2 m / 0.8 m).
     GICP output IS map_level <- camera_init. No alignment file is involved.
  5. Uniqueness by clustering the registration results: hypotheses closer
     than 1.0 m / 10 deg merge; if the best two DIFFERENT hypotheses have
     close quality the state is AMBIGUOUS and navigation stays blocked.
  6. VERIFYING requires VERIFY_COUNT consecutive frames that each pass
     converged + rmse + fitness + inlier gates. Only then LOCALIZED.

Outputs:
  - dynamic TF map_level -> camera_init (the ONLY publisher of that TF in
    localization mode)
  - runtime/localization_state.json with boot_id/sequence for the web UI
    and the motion bridge hard gate. Written first thing at startup as
    UNINITIALIZED so a stale LOCALIZED from a previous run can never be
    consumed.
"""
import json
import math
import os
import time
import uuid
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster

RUNTIME = Path(os.environ.get('PATROL_RUNTIME', '/project/runtime'))
DATA = Path(os.environ.get('PATROL_DATA', '/project/patrol_data'))

WINDOW_SEC = 4.0
WINDOW_MIN_POINTS = 2000
COARSE_VOXEL = 0.5
COARSE_CORR = 2.5
FINE_VOXEL = 0.2
FINE_CORR = 0.8
SUBMAP_RADIUS = 20.0
TRACK_RADIUS = 25.0
SEARCH_PERIOD = 1.0
MIN_INLIERS = 500

RMSE_MAX = float(os.environ.get('LOC_RMSE_MAX', '0.25'))
OVERLAP_MIN = float(os.environ.get('LOC_OVERLAP_MIN', '0.50'))
UNIQUE_MARGIN = float(os.environ.get('LOC_UNIQUE_MARGIN', '0.05'))
VERIFY_COUNT = 5
VERIFY_POS_TOL = 0.15
VERIFY_YAW_TOL = math.radians(5.0)
CORRECT_MAX = 0.30
CORRECT_YAW_MAX = math.radians(10.0)
LOST_CORRECT_MAX = 1.0
CLUSTER_POS = 1.0
CLUSTER_YAW = math.radians(10.0)


def angle_diff(a, b):
    """Absolute angular difference with wraparound (P0 audit: +/-180 deg)."""
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def yaw_of(t):
    return math.atan2(t[1, 0], t[0, 0])


def quat_matrix_from_msg(q):
    n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) or 1.0
    qx, qy, qz, qw = q.x / n, q.y / n, q.z / n, q.w / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def quat_matrix(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def voxel_downsample(points, voxel):
    if not len(points):
        return points
    keys = np.floor(points[:, :3] / voxel).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(index)]


def evaluate_registration(target, transformed, cell=0.3):
    """Self-computed quality metrics (small_gicp version-independent).

    fitness: fraction of transformed source points landing within `cell` of
    a target point (voxel-hash nearest-cell approximation).
    rmse: RMS of the distance from each transformed source point to the
    center of its nearest occupied target cell, over matched points.
    """
    if not len(target) or not len(transformed):
        return 0.0, 1e3
    t_keys = np.floor(target[:, :3] / cell).astype(np.int64)
    s_keys = np.floor(transformed[:, :3] / cell).astype(np.int64)
    t_map = {}
    for key in map(tuple, t_keys):
        t_map[key] = True
    s_unique, counts = np.unique(s_keys, axis=0, return_counts=True)
    hits = 0
    for key, count in zip(map(tuple, s_unique), counts):
        if key in t_map:
            hits += count
    fitness = hits / len(transformed) if len(transformed) else 0.0
    # RMSE via nearest occupied-cell center distance for a sample of points
    if t_map and len(transformed):
        sample = transformed[::max(1, len(transformed) // 2000)]
        t_arr = np.array(list(t_map.keys()), dtype=np.float64) * cell + cell / 2
        sq = []
        for p in sample:
            d2 = np.min(np.sum((t_arr - p) ** 2, axis=1))
            if d2 <= 0.5 ** 2 * 4:  # within 1 m
                sq.append(d2)
        rmse = math.sqrt(sum(sq) / len(sq)) if sq else 1e3
    else:
        rmse = 1e3
    return fitness, rmse


class Registration:
    """small_gicp backend (mandatory dependency, no silent fallback)."""

    def __init__(self, node):
        import small_gicp  # noqa: F401
        self.node = node
        node.get_logger().info('配准后端: small_gicp')

    def align(self, target, source, init_t, voxel, max_corr):
        import small_gicp
        target = np.ascontiguousarray(target[:, :3], dtype=np.float64)
        source = np.ascontiguousarray(source[:, :3], dtype=np.float64)
        result = small_gicp.align(
            target, source,
            init_T_target_source=np.asarray(init_t, dtype=np.float64),
            downsample_resolution=voxel,
            max_correspondence_distance=max_corr,
            num_threads=2)
        t = np.asarray(result.T_target_source, dtype=np.float64)
        transformed = source @ t[:3, :3].T + t[:3, 3]
        fitness, rmse = evaluate_registration(target, transformed)
        num_inliers = int(getattr(result, 'num_inliers', 0))
        converged = bool(getattr(result, 'converged', False))
        return {'T': t, 'converged': converged, 'num_inliers': num_inliers,
                'fitness': fitness, 'rmse': rmse,
                'quality_ok': (converged and fitness >= OVERLAP_MIN
                               and rmse <= RMSE_MAX
                               and num_inliers >= MIN_INLIERS)}


class LocalizationManager(Node):
    def __init__(self):
        super().__init__('localization_manager')
        self.boot_id = uuid.uuid4().hex
        self.sequence = 0
        self.state = 'UNINITIALIZED'
        self.state_message = '启动'
        self.map_id_est = None
        self.t_map_cam = None
        self.candidate = None
        self.verify_results = []
        self.degraded_count = 0
        self.last_margin = None
        self.last_rmse = None
        self.last_fitness = None
        self.window = []
        self.last_process = 0.0
        self.t_cam_sensor = np.eye(4)   # live odometry pose
        self.odo_seen = False
        self.load_maps()
        self.registration = Registration(self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, '/cloud_registered', self.on_cloud, qos)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0, self.tick)
        # First write MUST overwrite any stale state file from a previous run.
        self.write_state()
        self.get_logger().info(f'全局定位管理器就绪 boot_id={self.boot_id[:8]}')

    # ---------- data ----------
    def load_maps(self):
        for meta_path in sorted(DATA.glob('maps/*/metadata.json')):
            map_id = meta_path.parent.name
            meta = json.loads(meta_path.read_text(encoding='utf-8'))
            if not meta.get('localization_ready'):
                continue
            npy = meta_path.parent / 'map.npy'
            db = meta_path.parent / 'db.npz'
            if not npy.is_file() or not db.is_file():
                continue
            try:
                data = np.load(db)
                points = np.load(npy, mmap_mode='r')
                descriptors = data['descriptors']
                entries = []
                for i, d in enumerate(descriptors):
                    sc = d.reshape(tuple(data['sc_shape']))
                    entries.append({'sc': sc, 'ring_key': sc.mean(axis=1).astype(np.float32),
                                    'index': i, 'pose': data['poses'][i]})
                self.get_logger().info(
                    f'载入地图数据库: {map_id} 关键帧={len(entries)} 点云={points.shape[0]}')
                self.dbs[map_id] = {'entries': entries, 'points': points,
                                    'sc_shape': tuple(data['sc_shape'])}
            except (OSError, ValueError, KeyError) as exc:
                self.get_logger().warning(f'跳过地图 {map_id}: {exc}')

    # ---------- callbacks ----------
    def on_odom(self, msg):
        p = msg.pose.pose.position
        t = np.eye(4)
        t[:3, :3] = quat_matrix_from_msg(msg.pose.pose.orientation)
        t[:3, 3] = [p.x, p.y, p.z]
        self.t_cam_sensor = t
        self.odo_seen = True

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

    # ---------- window ----------
    def window_points(self):
        if not self.window or not self.odo_seen:
            return None
        now = time.monotonic()
        recent = [p for t, p in self.window if now - t <= WINDOW_SEC]
        if not recent:
            return None
        pts = np.vstack(recent)
        if len(pts) < WINDOW_MIN_POINTS:
            return None
        return voxel_downsample(pts, WINDOW_VOXEL)

    def sensor_frame(self, points):
        """camera_init world points -> current sensor frame (P0 audit #6)."""
        inv = np.linalg.inv(self.t_cam_sensor)
        return (np.ascontiguousarray(points, dtype=np.float64)
                @ inv[:3, :3].T + inv[:3, 3])

    # ---------- cycle ----------
    def tick(self):
        window = self.window_points()
        if window is None:
            if self.state in ('SEARCHING', 'UNINITIALIZED', 'LOST'):
                self.set_state('SEARCHING', '等待当前点云积累')
            self.publish_tf()
            self.write_state()
            return
        now = time.monotonic()
        if now - self.last_process < SEARCH_PERIOD:
            self.publish_tf()
            self.write_state()
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

    # ---------- search ----------
    def do_search(self, window):
        from patrol_global_localization.scan_context import (
            make_descriptor_height, search)
        query = make_descriptor_height(self.sensor_frame(window), sensor_z=0.0)
        all_candidates = []
        for map_name, bundle in self.dbs.items():
            try:
                top = search(bundle['entries'], query, topk=5)
            except Exception as exc:
                self.get_logger().warning(f'{map_name} 检索失败: {exc}')
                continue
            for entry, dist, shift in top:
                all_candidates.append({'map': map_name, 'entry': entry,
                                       'dist': dist, 'shift': shift})
        all_candidates.sort(key=lambda c: c['dist'])
        scored = []
        sector_angle = 2 * math.pi / self.dbs[all_candidates[0]['map']]['sc_shape'][1] \
            if all_candidates else 0.0
        for cand in all_candidates[:8]:
            pose = cand['entry']['pose']
            t_map_sensor_kf = np.eye(4)
            t_map_sensor_kf[:3, :3] = quat_matrix(*pose[3:7])
            t_map_sensor_kf[:3, 3] = pose[:3]
            # yaw shift from the sector match; sign verified by the
            # descriptor_yaw_shift_sign unit test
            yaw_fix = cand['shift'] * sector_angle
            init = (t_map_sensor_kf
                    @ transform_yaw(yaw_fix)
                    @ np.linalg.inv(self.t_cam_sensor))
            result = self.align_one(cand['map'], window, init)
            if result is None:
                continue
            scored.append({**cand, **result,
                           'x': float(result['T'][0, 3]),
                           'y': float(result['T'][1, 3]),
                           'yaw': yaw_of(result['T'])})
        if not scored:
            self.set_state('SEARCHING', '所有候选配准失败')
            return
        # Cluster hypotheses: position <1.0 m and yaw <10 deg merge.
        clusters = []
        for cand in sorted(scored, key=lambda c: -c['fitness']):
            for cluster in clusters:
                head = cluster[0]
                if (math.hypot(cand['x'] - head['x'], cand['y'] - head['y']) < CLUSTER_POS
                        and angle_diff(cand['yaw'], head['yaw']) < CLUSTER_YAW
                        and cand['map'] == head['map']):
                    cluster.append(cand)
                    break
            else:
                clusters.append([cand])
        clusters.sort(key=lambda c: -max(member['fitness'] for member in c))
        best_cluster = clusters[0]
        best = max(best_cluster, key=lambda c: c['fitness'])
        if len(clusters) > 1:
            second_best = max(clusters[1], key=lambda c: c['fitness'])
            margin = best['fitness'] - second_best['fitness']
            if margin < UNIQUE_MARGIN:
                self.set_state('AMBIGUOUS',
                               f'两个位置假设质量接近(边距{margin:.2f})，禁止导航')
                return
        self.last_margin = margin if len(clusters) > 1 else None
        if not best['quality_ok']:
            self.set_state('SEARCHING',
                           f"最佳候选质量不足(fitness={best['fitness']:.2f} "
                           f"rmse={best['rmse']:.2f})")
            return
        self.candidate = best
        self.verify_results = []
        self.set_state('VERIFYING',
                       f"候选 {best['map']}@({best['x']:.1f},{best['y']:.1f}) "
                       f"fitness={best['fitness']:.2f}")

    def do_verify(self, window):
        best = self.candidate
        result = self.align_one(best['map'], window, best['T'])
        if result is None or not result['quality_ok']:
            self.verify_results = []
            self.set_state('SEARCHING', '验证帧质量不足，重新检索')
            return
        t = result['T']
        consistent = all(
            np.linalg.norm(t[:3, 3] - prev['T'][:3, 3]) <= VERIFY_POS_TOL
            and angle_diff(yaw_of(t), yaw_of(prev['T'])) <= VERIFY_YAW_TOL
            for prev in self.verify_results)
        if not consistent:
            self.verify_results = [result]
            self.set_state('VERIFYING', '验证帧不一致，重新计数')
            return
        self.verify_results.append(result)
        if len(self.verify_results) >= VERIFY_COUNT:
            self.t_map_cam = t
            self.map_id_est = best['map']
            self.degraded_count = 0
            self.last_rmse = result['rmse']
            self.last_fitness = result['fitness']
            self.set_state('LOCALIZED',
                           f"定位成功 map={best['map']} "
                           f"fitness={result['fitness']:.2f}")
        else:
            self.set_state('VERIFYING', f'验证 {len(self.verify_results)}/{VERIFY_COUNT}')

    def do_track(self, window):
        result = self.align_one(self.map_id_est, window, self.t_map_cam)
        if result is None or not result['quality_ok']:
            self.degraded_count += 3
        else:
            t = result['T']
            jump = float(np.linalg.norm(t[:3, 3] - self.t_map_cam[:3, 3]))
            jump_yaw = angle_diff(yaw_of(t), yaw_of(self.t_map_cam))
            if jump > LOST_CORRECT_MAX or jump_yaw > math.radians(20):
                self.set_state('LOST', f'修正量异常({jump:.2f}m)，重新全局检索')
                self.degraded_count = 0
                self.t_map_cam = None
                return
            self.t_map_cam = t
            self.last_rmse = result['rmse']
            self.last_fitness = result['fitness']
            if (jump > CORRECT_MAX or jump_yaw > CORRECT_YAW_MAX):
                self.degraded_count += 1
            else:
                self.degraded_count = 0
        if self.degraded_count >= 6:
            self.set_state('LOST', '连续匹配失败，重新全局检索')
            self.degraded_count = 0
            self.t_map_cam = None
        elif self.degraded_count >= 3:
            self.set_state('DEGRADED', '匹配质量下降')

    def align_one(self, map_name, window, init_t):
        pts = self.dbs.get(map_name, {}).get('points')
        if pts is None:
            return None
        center = (np.asarray(init_t) @ np.array([0, 0, 0, 1.0]))[:3]
        d = np.hypot(pts[:, 0] - center[0], pts[:, 1] - center[1])
        submap = np.asarray(pts[d <= SUBMAP_RADIUS])
        if len(submap) < 1000:
            return None
        try:
            coarse = self.registration.align(submap, window, init_t,
                                             COARSE_VOXEL, COARSE_CORR)
            if not coarse['converged'] or coarse['fitness'] < 0.2:
                return None
            fine = self.registration.align(submap, window, coarse['T'],
                                           FINE_VOXEL, FINE_CORR)
            return fine
        except Exception as exc:
            self.get_logger().warning(f'配准异常: {exc}')
            return None

    # ---------- outputs ----------
    def publish_tf(self):
        if self.t_map_cam is None or self.state not in ('LOCALIZED', 'DEGRADED'):
            return
        # The GICP result IS map_level <- camera_init (map_level data end to
        # end); no alignment file is chained here.
        t = self.t_map_cam
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map_level'
        msg.child_frame_id = 'camera_init'
        msg.transform.translation.x, msg.transform.translation.y, \
            msg.transform.translation.z = t[0, 3], t[1, 3], t[2, 3]
        qx, qy, qz, qw = quat_from_matrix_pub(t[:3, :3])
        msg.transform.rotation.x, msg.transform.rotation.y = qx, qy
        msg.transform.rotation.z, msg.transform.rotation.w = qz, qw
        self.tf_broadcaster.sendTransform(msg)

    def set_state(self, state, message):
        if state != self.state:
            self.get_logger().info(f'定位状态: {self.state} -> {state} ({message})')
            self.state = state
        self.state_message = message

    def write_state(self):
        self.sequence += 1
        payload = {
            'state': self.state,
            'map_id': self.map_id_est,
            'boot_id': self.boot_id,
            'sequence': self.sequence,
            'message': self.state_message,
            'position': ([round(float(v), 3) for v in self.t_map_cam[:3, 3]]
                         if self.t_map_cam is not None else None),
            'yaw': round(float(yaw_of(self.t_map_cam)), 3) if self.t_map_cam is not None else None,
            'rmse': round(float(self.last_rmse), 3) if self.last_rmse else None,
            'fitness': round(float(self.last_fitness), 3) if self.last_fitness else None,
            'candidate_margin': round(float(self.last_margin), 3) if self.last_margin else None,
            'verified_frames': len(self.verify_results),
            'updated_at': time.time(),
            'ok_for_navigation': self.state == 'LOCALIZED',
        }
        tmp = Path(str(RUNTIME / 'localization_state.json') + '.tmp')
        tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=1),
                       encoding='utf-8')
        os.replace(tmp, RUNTIME / 'localization_state.json')


def quat_from_matrix_pub(r):
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
                (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(r)))
    if i == 0:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        return (0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s,
                (r[2, 1] - r[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        return ((r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s,
                (r[0, 2] - r[2, 0]) / s)
    s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
    return ((r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s,
            (r[1, 0] - r[0, 1]) / s)


def transform_yaw(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    t = np.eye(4)
    t[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    return t


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
