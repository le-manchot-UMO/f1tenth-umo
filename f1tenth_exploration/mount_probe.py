#!/usr/bin/env python3
"""Map close LiDAR returns into base_link XY to locate what is blocking the beam.

The laser sits at base_link + (0.27, 0, 0.11) with no rotation (bringup_launch.py).
A point at (range, angle) in the laser frame is therefore:
    x_base = 0.27 + range*cos(angle)
    y_base =        range*sin(angle)
Anything that stays put while the car moves is bolted to the car.
"""
import math, time, statistics
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

LX, LY = 0.27, 0.0
NEAR = 0.8

class P(Node):
    def __init__(self):
        super().__init__("mount_probe")
        self.scans = []
        self.create_subscription(LaserScan, "/scan", self.cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))
    def cb(self, m):
        if len(self.scans) < 30:
            self.scans.append(list(m.ranges)); self.m = m

def main():
    rclpy.init(); n = P()
    end = time.time() + 20
    while time.time() < end and len(n.scans) < 30:
        rclpy.spin_once(n, timeout_sec=0.2)
    if not n.scans: print("NO SCAN"); return
    m = n.m; N = len(n.scans[0])
    print(f"FOV {math.degrees(m.angle_min):.0f}..{math.degrees(m.angle_max):.0f} deg, "
          f"{N} beams, {len(n.scans)} scans")
    print("laser at base_link (0.27, 0.00, 0.11), identity rotation\n")

    # per-beam median range + stability
    beams = []
    for i in range(N):
        vals = [s[i] for s in n.scans if math.isfinite(s[i]) and s[i] > m.range_min]
        if len(vals) < len(n.scans) * 0.6: continue
        med = statistics.median(vals)
        sd = statistics.pstdev(vals) if len(vals) > 1 else 0.0
        if med < NEAR:
            a = m.angle_min + i * m.angle_increment
            beams.append((math.degrees(a), med, sd,
                          LX + med*math.cos(a), LY + med*math.sin(a)))
    if not beams:
        print(f"No returns under {NEAR} m — nothing near the sensor.")
        return

    # group into contiguous angular runs
    groups, cur = [], [beams[0]]
    for b in beams[1:]:
        if b[0] - cur[-1][0] <= 1.5: cur.append(b)
        else: groups.append(cur); cur = [b]
    groups.append(cur)

    print(f"{len(groups)} cluster(s) of returns under {NEAR} m:\n")
    for k, g in enumerate(groups, 1):
        a0, a1 = g[0][0], g[-1][0]
        rmin = min(x[1] for x in g); rmax = max(x[1] for x in g)
        xs = [x[3] for x in g]; ys = [x[4] for x in g]
        sd = statistics.mean([x[2] for x in g])
        print(f"cluster {k}: {a0:+.0f}..{a1:+.0f} deg ({len(g)} beams, {a1-a0:.0f} deg wide)")
        print(f"   range {rmin:.2f}..{rmax:.2f} m   mean stdev {sd:.4f} m")
        print(f"   base_link x {min(xs):+.2f}..{max(xs):+.2f} m,  y {min(ys):+.2f}..{max(ys):+.2f} m")
        # is it plausibly ON the car? car is ~ -0.12..+0.45 x, +-0.15 y
        on_car = (min(xs) > -0.30 and max(xs) < 0.70 and
                  min(ys) > -0.45 and max(ys) < 0.45)
        print(f"   {'WITHIN car bounds -> likely mounted hardware' if on_car else 'outside car bounds -> likely environment'}\n")
    n.destroy_node(); rclpy.shutdown()

main()
