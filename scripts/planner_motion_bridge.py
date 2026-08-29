#!/usr/bin/env python3
"""Fail-safe CMU-planner /cmd_vel to Unitree sport request bridge.

Adapted from nav_motion_bridge.py (Nav2/DWB version) for the new stack:
  /path   nav_msgs/Path   <- localPlanner   (was /plan from Nav2)
  /cmd_vel Twist          <- pathFollower   (same message, new producer)
  /terrain_map PointCloud2<- terrainAnalysis (was /scan LaserScan)

Safety interlocks kept identical: enable file, go2_state/robot_state
freshness, motor temperature, standing height, path-start proximity,
0.25 s command watchdog. The scan corridor is replaced by a terrain
corridor: /terrain_map is transformed into the body frame with the FULL
quaternion (the MID360 is mounted with ~35.6 deg pitch; a yaw-only
transform puts points in the wrong place), and obstacle voxels
(intensity ~= elevation above the voxel base, see probe_terrain.py)
inside the front corridor block forward motion. Terrain data going
stale BLOCKS motion (fail-closed), unlike the old fail-open scan check.

Pure-rotation commands from pathFollower's GOAL_ROT branch (x=0,
z!=0) are sent with the tiny-forward workaround Move(0.001,0,z) because
this GO2 firmware ignores Move(0,0,z). Set GO2_ROT_MODE=skip to drop
them instead, or GO2_ROT_MODE=direct to send them as-is (for firmware
that accepts pure yaw).
"""

import json
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, "/project/ros2_ws/src/patrol_global_localization")
from patrol_global_localization.navigation_gate import navigation_gate

import numpy as np

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener

ENABLE_FILE = Path(os.environ.get("NAV_ENABLE_FILE", "/project/runtime/nav_motion_enable.json"))
GO2_STATE_FILE = Path(os.environ.get("GO2_STATE_FILE", "/project/runtime/go2_state.json"))
ROBOT_STATE_FILE = Path(os.environ.get("ROBOT_STATE_FILE", "/project/runtime/robot_state.json"))
COMMAND_FILE = Path(os.environ.get("GO2_NAV_COMMAND_FILE", "/project/runtime/go2_nav_command.json"))
PATH_FILE = Path(os.environ.get("NAV_PATH_FILE", "/project/runtime/nav_path.json"))
MAX_MOTOR_TEMPERATURE_C = float(os.environ.get("MAX_MOTOR_TEMPERATURE_C", "85"))
MIN_STANDING_HEIGHT_M = float(os.environ.get("MIN_STANDING_HEIGHT_M", "0.18"))
# Rotation workaround mode: workaround (default) | direct | skip.
ROT_MODE = os.environ.get("GO2_ROT_MODE", "workaround")
# Terrain corridor: obstacle voxels closer than this in the front sector
# stop forward motion (mirrors the old 0.75 m /scan corridor).
FRONT_BLOCK_DIS = float(os.environ.get("FRONT_BLOCK_DIS", "0.75"))
FRONT_HALF_ANGLE_DEG = float(os.environ.get("FRONT_HALF_ANGLE_DEG", "35.0"))
# Obstacle voxels report intensity ~ vehicleHeight (0.4 m setup); use a
# slightly lower threshold so real obstacles trip before the max.
OBSTACLE_INTENSITY = float(os.environ.get("TERRAIN_OBSTACLE_INTENSITY", "0.36"))
TERRAIN_MAX_AGE = float(os.environ.get("TERRAIN_MAX_AGE", "1.5"))
def twist_fields(cloud, odom):
    """Return (front_min, left_min, right_min) obstacle distance in body frame.

    Same corridor logic as on_scan in the old bridge, but the source is the
    terrain map: transform each point world->body with the full quaternion
    from the latest odometry (the MID360 mount has ~35 deg pitch; a yaw-only
    transform misplaces points), then classify by body-frame yaw.  Vectorized
    with numpy — the per-point Python loop cost ~100% of one core at 10 Hz.
    """
    front = left = right = math.inf
    if odom is None:
        return front, left, right
    try:
        pts = point_cloud2.read_points_numpy(
            cloud, field_names=("x", "y", "z", "intensity"), skip_nans=True)
    except Exception:
        return front, left, right
    # read_points_numpy returns a plain (N, 4) array in field order.
    mask = pts[:, 3] >= OBSTACLE_INTENSITY
    if not mask.any():
        return front, left, right
    px = pts[mask, 0]
    py = pts[mask, 1]
    pz = pts[mask, 2]
    pose = odom.pose.pose
    q = pose.orientation
    # Full 3D rotation world->body: with the ~35.6 deg mount pitch, the
    # height difference projects into the horizontal body axes (~0.35 m at
    # 0.6 m height offset), so dropping the z term corrupts the corridor.
    xx = 1 - 2 * (q.y * q.y + q.z * q.z)
    xy = 2 * (q.x * q.y + q.z * q.w)
    xz = 2 * (q.x * q.z - q.y * q.w)
    yx = 2 * (q.x * q.y - q.z * q.w)
    yy = 1 - 2 * (q.x * q.x + q.z * q.z)
    yz = 2 * (q.y * q.z + q.x * q.w)
    tx, ty, tz = pose.position.x, pose.position.y, pose.position.z
    dx = px - tx
    dy = py - ty
    dz = pz - tz
    bx = xx * dx + xy * dy + xz * dz
    by = yx * dx + yy * dy + yz * dz
    distance = np.hypot(bx, by)
    near = distance <= 3.0
    if not near.any():
        return front, left, right
    yaw = np.arctan2(by[near], bx[near])
    d = distance[near]
    half = math.radians(FRONT_HALF_ANGLE_DEG)
    side = math.radians(125.0)
    f = d[abs(yaw) <= half]
    l = d[(yaw > half) & (yaw <= side)]
    r = d[(yaw < -half) & (yaw >= -side)]
    if f.size:
        front = float(f.min())
    if l.size:
        left = float(l.min())
    if r.size:
        right = float(r.min())
    return front, left, right


class PlannerMotionBridge(Node):
    def __init__(self):
        super().__init__("patrol_planner_motion_bridge")
        self.create_subscription(Twist, "/cmd_vel", self.on_velocity, 10)
        self.create_subscription(NavPath, "/path", self.on_path, 5)
        terrain_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, "/terrain_map", self.on_terrain, terrain_qos)
        odom_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, "/Odometry", self.on_odom, odom_qos)
        PATH_FILE.unlink(missing_ok=True)
        # Dynamic TF is the single source of truth (Plan A): route display is
        # transformed map_level <- camera_init via the live TF published by
        # the localization manager. No alignment-file fallback.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_command = 0.0
        self.front_min = math.inf
        self.left_min = math.inf
        self.right_min = math.inf
        self.last_terrain = 0.0
        self.odom = None
        self.path_start_ok = False
        self.last_plan = 0.0
        self.was_moving = False
        self.create_timer(0.05, self.watchdog)
        self.get_logger().info(
            f"Planner motion bridge ready (rot_mode={ROT_MODE}); motion disabled until enabled")

    def on_odom(self, message):
        self.odom = message

    def on_terrain(self, message):
        self.front_min, self.left_min, self.right_min = twist_fields(message, self.odom)
        self.last_terrain = time.monotonic()

    def on_path(self, message):
        poses = message.poses
        stride = max(1, math.ceil(len(poses) / 500))
        # localPlanner publishes in base_footprint (vehicle frame, heading=+x);
        # lift it into camera_init via the live odometry, then into map_level
        # with the same level transform the cloud uses, so the web can draw
        # the route on the 3D map instead of offset from it.
        vehicle_frame = (message.header.frame_id or "") == "base_footprint"
        points = []
        try:
            m = tf_buffer_lookup(self.tf_buffer, "map_level", "camera_init")
        except Exception:
            m = None
        if m is None:
            # No live localization TF: publishing a route in a guessed frame
            # would draw it in the wrong place — skip this cycle.
            return
        t_cam_vehicle = np.eye(4)
        if self.odom is not None:
            opose = self.odom.pose.pose
            t_cam_vehicle[:3, :3] = quat_matrix_from_odom(opose.orientation)
            t_cam_vehicle[:3, 3] = [opose.position.x, opose.position.y,
                                    opose.position.z]
        full = m @ t_cam_vehicle
        for p in poses[::stride]:
            v = full @ np.array([float(p.pose.position.x), float(p.pose.position.y),
                                 float(p.pose.position.z), 1.0])
            points.append([round(float(v[0]), 3), round(float(v[1]), 3),
                           round(float(v[2]), 3)])
        temporary = Path(f"{PATH_FILE}.{os.getpid()}.tmp")
        payload = {"available": bool(points), "frame": "map_level",
                   "updated_at": time.time(), "points": points}
        temporary.write_text(json.dumps(payload, ensure_ascii=False,
                                        separators=(",", ":")) + "\n", encoding="utf-8")
        os.replace(temporary, PATH_FILE)
        self.last_plan = time.monotonic()
        # /path from localPlanner is in the base_footprint (vehicle) frame: it
        # always starts at (0,0) relative to the robot, so the old Nav2-era
        # endpoint-vs-odometry check is meaningless here (it even rejects
        # valid paths once the robot leaves the FAST-LIO origin). Path
        # freshness (last_plan, checked in enabled()) is the real gate.
        self.path_start_ok = bool(points)

    def localization_ready(self):
        """Plan A hard gate, FAIL-CLOSED (P0 audit #11): GO2 motion requires
        a fresh localization_state.json reporting LOCALIZED for the active
        map. A missing file means the localizer is not running — that is a
        stop condition, never a bypass."""
        path = Path("/project/runtime/localization_state.json")
        try:
            loc = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            self.get_logger().error("localization_state.json 缺失：定位未运行，禁止运动",
                                    throttle_duration_sec=5.0)
            return False
        if time.time() - float(loc.get("updated_at", 0)) > 2.0:
            self.get_logger().error("定位状态过期，禁止运动", throttle_duration_sec=5.0)
            return False
        if loc.get("state") != "LOCALIZED" or not loc.get("ok_for_navigation"):
            return False
        active_map = json.loads(Path("/project/patrol_data/active_map.json")
                                .read_text(encoding="utf-8")).get("name")
        if active_map and loc.get("map_id") != active_map:
            self.get_logger().error(
                f"定位地图({loc.get('map_id')})与活动地图({active_map})不一致，禁止运动",
                throttle_duration_sec=5.0)
            return False
        return True

    def enabled(self):
        try:
            state = json.loads(ENABLE_FILE.read_text(encoding="utf-8"))
            safety = json.loads(GO2_STATE_FILE.read_text(encoding="utf-8"))
            robot = json.loads(ROBOT_STATE_FILE.read_text(encoding="utf-8"))
            fresh = (time.time() - float(safety.get("updated_at", 0)) < 2.0
                     and time.time() - float(robot.get("updated_at", 0)) < 2.0)
            cool = float(safety.get("max_motor_temperature_c", 999)) < MAX_MOTOR_TEMPERATURE_C
            standing = float(safety.get("body_height", 0)) >= MIN_STANDING_HEIGHT_M
            # nav_status=="navigating" was the Nav2 action lifecycle gate; the
            # planner stack has no Nav2, so gate on sane localization instead.
            sane = robot.get("localization_sane") is not False
            route_valid = self.path_start_ok and time.monotonic() - self.last_plan < 3.0
            if not self.localization_ready():
                return False
            return (bool(state.get("enabled")) and time.time() < float(state.get("expires_at", 0))
                    and fresh and cool and standing and sane and route_valid)
        except (OSError, ValueError, TypeError):
            return False

    def publish_sport(self, api_id, parameter=""):
        # Unique tmp name: the web server writes the same command file from
        # the host; sharing one ".tmp" makes the two os.replace calls race.
        temporary = Path(f"{COMMAND_FILE}.{os.getpid()}.tmp")
        payload = {"api_id": api_id, "parameter": parameter,
                   "source": "navigation", "updated_at": time.time()}
        temporary.write_text(json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n", encoding="utf-8")
        os.replace(temporary, COMMAND_FILE)

    def stop(self):
        if self.was_moving:
            self.publish_sport(1003)
        self.was_moving = False

    @staticmethod
    def effective(value, minimum):
        if abs(value) <= 0.005:
            return 0.0
        return math.copysign(max(abs(value), minimum), value)

    def on_velocity(self, message):
        self.last_command = time.monotonic()
        if not self.enabled():
            self.stop()
            return
        raw_x = max(-0.50, min(0.50, float(message.linear.x)))
        raw_y = max(-0.50, min(0.50, float(message.linear.y)))
        raw_z = max(-0.85, min(0.85, float(message.angular.z)))
        vx = self.effective(raw_x, 0.20)
        vy = self.effective(raw_y, 0.20)
        vyaw = 0.0 if abs(raw_z) < 0.05 else raw_z

        terrain_fresh = time.monotonic() - self.last_terrain < TERRAIN_MAX_AGE
        front_blocked = terrain_fresh and self.front_min < FRONT_BLOCK_DIS
        if not terrain_fresh:
            # Fail-closed: without a current terrain map do not translate.
            self.get_logger().warn("terrain_map stale; blocking motion", throttle_duration_sec=2.0)
            self.stop()
            return

        if abs(vy) > 0.005 and front_blocked:
            if self.left_min >= 0.65 and self.left_min > self.right_min + 0.15:
                vy = abs(vy)
            elif self.right_min >= 0.65 and self.right_min > self.left_min + 0.15:
                vy = -abs(vy)
        if abs(vx) > 0.005 and abs(vy) > 0.005:
            if front_blocked:
                vx = 0.0
            else:
                vy = 0.0
        elif front_blocked and abs(vx) > 0.005:
            vx = 0.0

        # Rotation: pathFollower's GOAL_ROT branch asks for x=0, z!=0. Pure
        # rotation is ignored by this firmware, but go2_state_bridge now
        # rewrites any rotate-only request into a tight arc (x=0.10, measured
        # 2026-08-29, turn radius ~0.15 m). So pass it through unchanged.
        if abs(vyaw) > 0.005 and abs(vx) <= 0.005 and abs(vy) <= 0.005:
            if ROT_MODE == "skip":
                self.stop()
                return
            self.publish_sport(1008, json.dumps(
                {"x": 0.0, "y": 0.0, "z": vyaw}, separators=(",", ":")))
            self.was_moving = True
            return

        # Continuous small-id streams accept native arcs Move(x,y,z), so keep
        # the follower's yaw component instead of serializing the axes (the
        # old z-drop existed only for the pulsed-cadence workaround and made
        # the path following oscillate: drive straight, then turn).
        if any(abs(value) > 0.005 for value in (vx, vy)):
            self.publish_sport(1008, json.dumps({"x": vx, "y": vy, "z": vyaw}, separators=(",", ":")))
            self.was_moving = True
        else:
            self.stop()

    def watchdog(self):
        if not self.enabled() or (self.was_moving and time.monotonic() - self.last_command > 0.25):
            self.stop()


def main():
    rclpy.init()
    node = PlannerMotionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

def quat_matrix_from_odom(q):
    n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) or 1.0
    qx, qy, qz, qw = q.x / n, q.y / n, q.z / n, q.w / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def tf_buffer_lookup(buffer, target, source):
    """Latest TF as a 4x4 matrix; raises if the transform is unavailable."""
    m = buffer.lookup_transform(target, source, Time())
    tr = m.transform
    matrix = np.eye(4)
    matrix[:3, :3] = quat_matrix_from_odom(tr.rotation)
    matrix[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
    return matrix
