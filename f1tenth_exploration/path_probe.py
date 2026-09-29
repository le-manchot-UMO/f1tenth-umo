#!/usr/bin/env python3
"""Why won't the controller follow the plan? Sample costmap cost ALONG the path.

Nav2 aborting with "RegulatedPurePursuitController detected collision ahead!" while
the planner happily produces a path means the two disagree: the global planner
routes through cells the controller's footprint check rejects. This walks the
current /plan and reports the cost under the robot footprint at each point, so you
can see exactly where and why it refuses.

Cost scale on the published costmap is 0-100 (100 = lethal). Cells within the
INSCRIBED radius of an obstacle come out at ~99-100 and are untraversable.
"""
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid, Path
import tf2_ros

# footprint corners in base_link (must match nav2_params.yaml)
FOOTPRINT = [(0.45, 0.14), (0.45, -0.14), (-0.11, -0.14), (-0.11, 0.14)]

class P(Node):
    def __init__(self):
        super().__init__("path_probe")
        self.plan = None; self.local = None; self.glob = None
        self.buf = tf2_ros.Buffer(); tf2_ros.TransformListener(self.buf, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap",
                                 lambda m: setattr(self, "local", m), qos)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 lambda m: setattr(self, "glob", m), qos)
        self.create_subscription(Path, "/plan", lambda m: setattr(self, "plan", m), 10)

def cost_at(grid, x, y):
    i = grid.info
    cx = int((x - i.origin.position.x) / i.resolution)
    cy = int((y - i.origin.position.y) / i.resolution)
    if 0 <= cx < i.width and 0 <= cy < i.height:
        return grid.data[cy * i.width + cx]
    return None

def footprint_max(grid, x, y, yaw):
    worst = -1
    for dx, dy in FOOTPRINT:
        px = x + dx*math.cos(yaw) - dy*math.sin(yaw)
        py = y + dx*math.sin(yaw) + dy*math.cos(yaw)
        c = cost_at(grid, px, py)
        if c is not None:
            worst = max(worst, c)
    return worst

def main():
    rclpy.init(); n = P()
    end = time.time() + 25
    while time.time() < end and (n.plan is None or n.local is None):
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.plan is None:
        print("NO /plan — the planner is not producing a path"); return
    if n.local is None:
        print("NO local costmap"); return
    poses = n.plan.poses
    print(f"plan: {len(poses)} poses, frame {n.plan.header.frame_id}")
    if not poses: return

    try:
        t = n.buf.lookup_transform("odom", "base_link", rclpy.time.Time())
        print(f"robot at odom ({t.transform.translation.x:+.2f}, {t.transform.translation.y:+.2f})")
    except Exception:
        pass

    print(f"\n{'idx':>4} {'dist m':>7} {'local':>6} {'global':>7}  footprint verdict")
    blocked = 0
    prev = None; dist = 0.0
    step = max(1, len(poses)//25)
    for k in range(0, len(poses), step):
        ps = poses[k].pose
        x, y = ps.position.x, ps.position.y
        q = ps.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if prev: dist += math.hypot(x-prev[0], y-prev[1])
        prev = (x, y)
        lc = footprint_max(n.local, x, y, yaw)
        gc = footprint_max(n.glob, x, y, yaw) if n.glob else -1
        verdict = ""
        if lc >= 99: verdict = "<-- LETHAL in local costmap"; blocked += 1
        elif lc >= 90: verdict = "<-- near-lethal"
        print(f"{k:4d} {dist:7.2f} {lc:6d} {gc:7d}  {verdict}")
    print(f"\n{blocked} of the sampled points put the FOOTPRINT in lethal local cost.")
    if blocked:
        print("The planner routed through cells the controller's footprint check")
        print("rejects. Usual causes: footprint too large, inflation too wide, or")
        print("the global costmap is staler than the local one.")
    n.destroy_node(); rclpy.shutdown()

main()
