#!/usr/bin/env python3
"""Save /map and both costmaps as PNGs, plus a polar plot of the live scan."""
import math, os, sys, time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/maps"
os.makedirs(OUT, exist_ok=True)
STAMP = time.strftime("%Y%m%d_%H%M%S")

class Snap(Node):
    def __init__(self):
        super().__init__("map_snapshot")
        self.g = {}
        self.scan = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        for t in ["/map", "/global_costmap/costmap", "/local_costmap/costmap"]:
            self.create_subscription(OccupancyGrid, t, lambda m, t=t: self.g.setdefault(t, m), qos)
        self.create_subscription(LaserScan, "/scan", self.s_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))
    def s_cb(self, m):
        if self.scan is None: self.scan = m

def grid_to_png(msg, title, path, costmap):
    w, h = msg.info.width, msg.info.height
    a = np.array(msg.data, dtype=np.int16).reshape(h, w)
    res = msg.info.resolution
    ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
    extent = [ox, ox + w * res, oy, oy + h * res]

    fig, ax = plt.subplots(figsize=(9, 9 * h / max(w, 1) + 1.1), dpi=130)
    if costmap:
        # -1 unknown, 0 free, 1..98 gradient, 99 inscribed, 100 lethal
        disp = np.where(a < 0, np.nan, a).astype(float)
        im = ax.imshow(disp, origin="lower", extent=extent, cmap="turbo",
                       vmin=0, vmax=100, interpolation="nearest")
        fig.colorbar(im, ax=ax, shrink=.75, label="cost (100 = lethal)")
        ax.set_facecolor("#585858")   # unknown
    else:
        cmap = ListedColormap(["#5A5A5A", "#F2F2F2", "#101010"])
        norm = BoundaryNorm([-1.5, -0.5, 50, 101], cmap.N)
        ax.imshow(a, origin="lower", extent=extent, cmap=cmap, norm=norm,
                  interpolation="nearest")
    occ = int((a > 50).sum()); unk = int((a < 0).sum()); tot = a.size
    ax.set_title(f"{title}\n{w}x{h} @ {res:.3f} m/cell  |  "
                 f"unknown {100*unk/tot:.1f}%  occupied {100*occ/tot:.1f}%", fontsize=11)
    ax.set_xlabel(f"x [m] ({msg.header.frame_id})"); ax.set_ylabel("y [m]")
    ax.grid(alpha=.18, linewidth=.5)
    fig.tight_layout(); fig.savefig(path, bbox_inches="tight"); plt.close(fig)
    print(f"  {path}  ({w}x{h}, unknown {100*unk/tot:.1f}%, occupied {100*occ/tot:.1f}%)")

def scan_to_png(m, path):
    n = len(m.ranges)
    ang = np.array([m.angle_min + i * m.angle_increment for i in range(n)])
    r = np.array(m.ranges, dtype=float)
    ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
    fig = plt.figure(figsize=(7.6, 7.6), dpi=130)
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N"); ax.set_theta_direction(-1)
    close = ok & (r < 0.6)
    ax.scatter(ang[ok & ~close], r[ok & ~close], s=2.2, c="#1f6feb", label="returns")
    ax.scatter(ang[close], r[close], s=9, c="#d62828", label="< 0.6 m (suspect self-hit)")
    ax.set_rmax(min(6.0, float(np.nanmax(r[ok])) if ok.any() else 6.0))
    ax.set_title(f"LaserScan, car frame\n{math.degrees(m.angle_min):.0f}° to "
                 f"{math.degrees(m.angle_max):.0f}°, {n} beams", fontsize=11, pad=18)
    ax.legend(loc="lower left", bbox_to_anchor=(-.12, -.12), fontsize=9)
    fig.tight_layout(); fig.savefig(path, bbox_inches="tight"); plt.close(fig)
    print(f"  {path}")

def main():
    rclpy.init(); n = Snap()
    end = time.time() + 25
    while time.time() < end and (len(n.g) < 3 or n.scan is None):
        rclpy.spin_once(n, timeout_sec=0.25)
    print(f"snapshot {STAMP} -> {OUT}")
    for topic, name, cm in [("/map", "SLAM map", False),
                            ("/global_costmap/costmap", "Global costmap", True),
                            ("/local_costmap/costmap", "Local costmap", True)]:
        if topic in n.g:
            grid_to_png(n.g[topic], name, f"{OUT}/{STAMP}_{name.lower().replace(' ','_')}.png", cm)
        else:
            print(f"  MISSING {topic}")
    if n.scan is not None:
        scan_to_png(n.scan, f"{OUT}/{STAMP}_laserscan.png")
    n.destroy_node(); rclpy.shutdown()

main()
