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
ENABLE_FILE = Path(os.environ.get("NAV_ENABLE_FILE", "/project/runtime/nav_motion_enable.json"))
GO2_STATE_FILE = Path(os.environ.get("GO2_STATE_FILE", "/project/runtime/go2_state.json"))
COMMAND_FILE = Path(os.environ.get("GO2_NAV_COMMAND_FILE", "/project/runtime/go2_nav_command.json"))
PATH_FILE = Path(os.environ.get("NAV_PATH_FILE", "/project/runtime/nav_path.json"))
MAX_MOTOR_TEMPERATURE_C = float(os.environ.get("MAX_MOTOR_TEMPERATURE_C", "85"))
MIN_STANDING_HEIGHT_M = float(os.environ.get("MIN_STANDING_HEIGHT_M", "0.18"))


class NavMotionBridge(Node):
    def __init__(self):
        super().__init__("patrol_nav_motion_bridge")
        self.create_subscription(Twist, "/cmd_vel", self.on_velocity, 10)
        self.create_subscription(NavPath, "/plan", self.on_plan, 10)
        PATH_FILE.unlink(missing_ok=True)
        self.last_command = 0.0
        self.was_moving = False
        self.create_timer(0.05, self.watchdog)
        self.get_logger().info("Navigation motion bridge ready; motion remains disabled until confirmed")

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

    def enabled(self):
        try:
            state = json.loads(ENABLE_FILE.read_text(encoding="utf-8"))
            robot = json.loads(GO2_STATE_FILE.read_text(encoding="utf-8"))
            fresh = time.time() - float(robot.get("updated_at", 0)) < 2.0
            cool = float(robot.get("max_motor_temperature_c", 999)) < MAX_MOTOR_TEMPERATURE_C
            standing = float(robot.get("body_height", 0)) >= MIN_STANDING_HEIGHT_M
            return (bool(state.get("enabled")) and time.time() < float(state.get("expires_at", 0))
                    and fresh and cool and standing)
        except (OSError, ValueError, TypeError):
            return False

    def publish_sport(self, api_id, parameter=""):
        temporary = Path(str(COMMAND_FILE) + ".tmp")
        payload = {"api_id": api_id, "parameter": parameter, "updated_at": time.time()}
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
        vx = self.effective(max(-0.20, min(0.20, float(message.linear.x))), 0.20)
        # DWB uses GO2's holonomic side-step to follow the path without first
        # requiring an in-place rotation.  The verified web-control threshold
        # for lateral walking is the same 0.20 m/s used for forward walking.
        vy = self.effective(max(-0.20, min(0.20, float(message.linear.y))), 0.20)
        raw_yaw = max(-0.80, min(0.80, float(message.angular.z)))
        # Do not turn a tiny straight-line correction into a continuous turn.
        # The GO2 starts walking more reliably when near-straight RPP paths are
        # sent as pure forward commands.
        # Pulsed yaw below about 0.4 rad/s barely moves this GO2.  Preserve the
        # Nav2 sign but raise rotate-only requests to the verified threshold.
        # RPP deliberately starts a heading correction near 0.03--0.04 rad/s
        # when odometry reports zero angular speed.  Dropping everything below
        # 0.08 made that startup command disappear forever, so the robot could
        # never begin the turn and the controller could never ramp up.
        # 0.4 rad/s produced mostly body sway on this GO2.  A deliberate
        # 0.8 rad/s rotate-only request is needed for visible foot rotation;
        # translation remains zero until RPP reports the heading aligned.
        vyaw = self.effective(raw_yaw, 0.80)
        # Match the proven web-control semantics: GO2 receives one motion axis
        # at a time.  While Nav2 requests translation, suppress simultaneous
        # yaw; a later rotate-only command is kept for heading correction.
        if abs(vx) > 0.005 or abs(vy) > 0.005:
            vyaw = 0.0
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
