#!/usr/bin/env python3
"""Wait for /start_far_planner and call it (retry until accepted).

far_planner gates its planning loop behind this Trigger service; without
the call it silently drops every /goal_point. Retries for up to 90 s so
it can run right after the stack launch.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

rclpy.init()
node = Node("far_auto_start")
cli = node.create_client(Trigger, "/start_far_planner")

deadline = time.monotonic() + 90.0
ok = False
while time.monotonic() < deadline:
    if cli.wait_for_service(timeout_sec=2.0):
        future = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if future.result() is not None and future.result().success:
            ok = True
            break
    time.sleep(1.0)

print("start_far_planner: OK" if ok else "start_far_planner: FAILED (timeout)")

# is_stop_update_ defaults to TRUE: environment / v-graph updates stay
# frozen until /resume_visibility_graph_update is called.  Without it the
# graph never initialises and every /goal_point is silently dropped.
if ok:
    cli2 = node.create_client(Trigger, "/resume_visibility_graph_update")
    if cli2.wait_for_service(timeout_sec=5.0):
        future = cli2.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        print("resume_vgraph: OK" if future.result() is not None else "resume_vgraph: no reply")
    else:
        print("resume_vgraph: service unavailable")
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
