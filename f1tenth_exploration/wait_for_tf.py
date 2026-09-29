#!/usr/bin/env python3
"""
Block until a TF `parent -> child` is available, then exit 0. Exits 1 on timeout.

Used by launch_f1tenth_exploration.sh so the Nav2 stack doesn't launch (and bring
its costmaps up BLIND) before vesc_to_odom is publishing odom -> base_link. The
costmaps log "Timed out waiting for transform from base_link to odom ... two or
more unconnected trees" hundreds of times when this TF is missing, which means
no scan raytracing -> no obstacles -> the car drives into walls.

Usage:
    python3 wait_for_tf.py <parent_frame> <child_frame> [timeout_sec]
        timeout_sec defaults to 120. Exits 0 once the TF is seen, 1 on timeout.
"""
import sys

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    if len(sys.argv) < 3:
        print("usage: wait_for_tf.py <parent> <child> [timeout_sec]", file=sys.stderr)
        sys.exit(2)
    parent = sys.argv[1]
    child = sys.argv[2]
    try:
        timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 120.0
    except ValueError:
        print(f"invalid timeout: {sys.argv[3]}", file=sys.stderr)
        sys.exit(2)

    rclpy.init()
    node = Node("wait_for_tf")
    buf = Buffer()
    TransformListener(buf, node)

    import time
    deadline = time.time() + timeout
    print(f"Waiting for TF {parent} -> {child} (up to {timeout:.0f}s)...", flush=True)
    found = False
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            buf.lookup_transform(parent, child, rclpy.time.Time())
            found = True
            break
        except TransformException:
            pass

    node.destroy_node()
    rclpy.shutdown()
    if found:
        print(f"TF {parent} -> {child} is available.", flush=True)
        sys.exit(0)
    print(f"TIMEOUT: TF {parent} -> {child} not available after {timeout:.0f}s.",
          file=sys.stderr, flush=True)
    sys.exit(1)


if __name__ == "__main__":
    main()