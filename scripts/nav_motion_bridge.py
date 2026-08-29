#!/usr/bin/env python3
"""Fail-safe Nav2 cmd_vel to Unitree sport request bridge."""

import json
import math
import os
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
ENABLE_FILE = Path(os.environ.get("NAV_ENABLE_FILE", "/project/runtime/nav_motion_enable.json"))
GO2_STATE_FILE = Path(os.environ.get("GO2_STATE_FILE", "/project/runtime/go2_state.json"))
ROBOT_STATE_FILE = Path(os.environ.get("ROBOT_STATE_FILE", "/project/runtime/robot_state.json"))
COMMAND_FILE = Path(os.environ.get("GO2_NAV_COMMAND_FILE", "/project/runtime/go2_nav_command.json"))
PATH_FILE = Path(os.environ.get("NAV_PATH_FILE", "/project/runtime/nav_path.json"))
MAX_MOTOR_TEMPERATURE_C = float(os.environ.get("MAX_MOTOR_TEMPERATURE_C", "85"))
MIN_STANDING_HEIGHT_M = float(os.environ.get("MIN_STANDING_HEIGHT_M", "0.18"))


class NavMotionBridge(Node):
    def __init__(self):
        super().__init__("patrol_nav_motion_bridge")
        self.create_subscription(Twist, "/cmd_vel", self.on_velocity, 10)
        self.create_subscription(NavPath, "/plan", self.on_plan, 10)
        scan_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, "/scan", self.on_scan, scan_qos)
        PATH_FILE.unlink(missing_ok=True)
        self.last_command = 0.0
        self.front_min = math.inf
        self.left_min = math.inf
        self.right_min = math.inf
        self.last_scan = 0.0
        self.path_start_ok = False
        self.last_plan = 0.0
        self.was_moving = False
        self.create_timer(0.05, self.watchdog)
        self.get_logger().info("Navigation motion bridge ready; motion remains disabled until confirmed")

    def on_scan(self, message):
        """Keep the nearest valid return in a 70-degree forward corridor."""
        nearest = math.inf
        left = math.inf
        right = math.inf
        angle = float(message.angle_min)
        for distance in message.ranges:
            wrapped = math.atan2(math.sin(angle), math.cos(angle))
            if abs(wrapped) <= math.radians(35.0) and math.isfinite(distance):
                if message.range_min <= distance <= message.range_max:
                    nearest = min(nearest, float(distance))
            elif math.radians(35.0) < wrapped <= math.radians(125.0) and math.isfinite(distance):
                if message.range_min <= distance <= message.range_max:
                    left = min(left, float(distance))
            elif math.radians(-125.0) <= wrapped < math.radians(-35.0) and math.isfinite(distance):
                if message.range_min <= distance <= message.range_max:
                    right = min(right, float(distance))
            angle += float(message.angle_increment)
        self.front_min = nearest
        self.left_min = left
        self.right_min = right
        self.last_scan = time.monotonic()

    def on_plan(self, message):
        poses = message.poses
        stride = max(1, math.ceil(len(poses) / 500))
        points = [[float(p.pose.position.x), float(p.pose.position.y),
                   float(p.pose.position.z)] for p in poses[::stride]]
        temporary = Path(str(PATH_FILE) + ".tmp")
        payload = {"available": bool(points), "frame": message.header.frame_id or "map_level",
                   "updated_at": time.time(), "points": points}
        temporary.write_text(json.dumps(payload, ensure_ascii=False,
                                        separators=(",", ":")) + "\n", encoding="utf-8")
        os.replace(temporary, PATH_FILE)
        self.last_plan = time.monotonic()
        self.path_start_ok = False
        if points:
            try:
                robot = json.loads(ROBOT_STATE_FILE.read_text(encoding="utf-8"))
                pose = robot.get("pose", {})
                error = math.hypot(points[0][0] - float(pose["x"]),
                                   points[0][1] - float(pose["y"]))
                self.path_start_ok = error <= 0.35
                if not self.path_start_ok:
                    self.get_logger().error(
                        f"Rejecting path: start is {error:.2f} m from current robot pose")
            except (OSError, ValueError, TypeError, KeyError):
                self.path_start_ok = False

    def enabled(self):
        try:
            state = json.loads(ENABLE_FILE.read_text(encoding="utf-8"))
            safety = json.loads(GO2_STATE_FILE.read_text(encoding="utf-8"))
            robot = json.loads(ROBOT_STATE_FILE.read_text(encoding="utf-8"))
            fresh = (time.time() - float(safety.get("updated_at", 0)) < 2.0
                     and time.time() - float(robot.get("updated_at", 0)) < 2.0)
            cool = float(safety.get("max_motor_temperature_c", 999)) < MAX_MOTOR_TEMPERATURE_C
            standing = float(safety.get("body_height", 0)) >= MIN_STANDING_HEIGHT_M
            route_valid = self.path_start_ok and time.monotonic() - self.last_plan < 3.0
            return (bool(state.get("enabled")) and time.time() < float(state.get("expires_at", 0))
                    and fresh and cool and standing
                    and robot.get("nav_status") == "navigating" and route_valid)
        except (OSError, ValueError, TypeError):
            return False

    def publish_sport(self, api_id, parameter=""):
        temporary = Path(str(COMMAND_FILE) + ".tmp")
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
        # Nav2 already shapes the velocity.  Keep only a small dead-band here;
        # large minimums turn tiny path corrections into left/right oscillation.
        # Use the same 0.20 m/s motion level verified by the direct web control.
        # Lower 0.10 m/s Nav2 requests were published correctly but did not make
        # this GO2 firmware start walking after reboot.
        raw_x = max(-0.20, min(0.20, float(message.linear.x)))
        raw_y = max(-0.20, min(0.20, float(message.linear.y)))
        vx = self.effective(raw_x, 0.20)
        # DWB uses GO2's holonomic side-step to follow the path without first
        # requiring an in-place rotation.  The verified web-control threshold
        # for lateral walking is the same 0.20 m/s used for forward walking.
        vy = self.effective(raw_y, 0.20)
        vyaw = 0.0

        # This GO2 firmware accepts forward and lateral Sport Move commands,
        # but a verified pure-yaw command does not rotate the robot.  Serialize
        # DWB's holonomic command into one physical axis: while an obstacle is
        # close ahead, side-step along the planner-selected sign; once the
        # forward corridor is clear, suppress lateral correction and advance.
        scan_fresh = time.monotonic() - self.last_scan < 0.5
        front_blocked = scan_fresh and self.front_min < 0.75
        if front_blocked and abs(vy) > 0.005:
            # DWB may choose a geometrically short side even when the immediate
            # physical clearance is worse.  Prefer the side with at least
            # 0.15 m more measured room; ROS body +Y is left.
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
            # Never drive straight into a return inside the safety corridor.
            vx = 0.0
        if any(abs(value) > 0.005 for value in (vx, vy, vyaw)):
            self.publish_sport(1008, json.dumps({"x": vx, "y": vy, "z": vyaw}, separators=(",", ":")))
            self.was_moving = True
        else:
            self.stop()

    def watchdog(self):
        if not self.enabled() or (self.was_moving and time.monotonic() - self.last_command > 0.25):
            self.stop()


def main():
    rclpy.init()
    node = NavMotionBridge()
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
