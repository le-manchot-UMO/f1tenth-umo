#!/usr/bin/env python3
"""Record the SLAM map and both costmaps as video frames during a run.

Writes a numbered PNG per tick showing all three grids side by side with the
car's track drawn on top. The car has no ffmpeg, so frames are encoded to MP4
afterwards on a machine that does:

    scp -r f1-prime@<car>:~/f1tenth_video/<run> .
    ffmpeg -framerate 10 -i <run>/frame_%05d.png -c:v libx264 \
           -pix_fmt yuv420p -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" run.mp4

Usage: map_recorder.py [outdir] [period_seconds]
"""
import os
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import tf2_ros
from matplotlib.colors import BoundaryNorm, ListedColormap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/f1tenth_video/run")
PERIOD = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0

TOPICS = [("/map", "SLAM map", False),
          ("/global_costmap/costmap", "Global costmap", True),
          ("/local_costmap/costmap", "Local costmap", True)]


class Recorder(Node):
    def __init__(self):
        super().__init__("map_recorder")
        self.grids = {}
        self.track = []
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        for topic, _, _ in TOPICS:
            self.create_subscription(OccupancyGrid, topic,
                                     lambda m, t=topic: self.grids.__setitem__(t, m), qos)

    def pose(self):
        """Car position in the map frame, or None until TF is up."""
        try:
            t = self.buf.lookup_transform("map", "base_link", rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None


def draw(ax, msg, title, costmap, track):
    if msg is None:
        ax.text(.5, .5, "waiting…", ha="center", va="center",
                transform=ax.transAxes, color="#888")
        ax.set_title(title, fontsize=10)
        ax.set_xticks([]); ax.set_yticks([])
        return
    w, h = msg.info.width, msg.info.height
    a = np.array(msg.data, dtype=np.int16).reshape(h, w)
    res = msg.info.resolution
    ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
    extent = [ox, ox + w * res, oy, oy + h * res]
    if costmap:
        ax.imshow(np.where(a < 0, np.nan, a).astype(float), origin="lower",
                  extent=extent, cmap="turbo", vmin=0, vmax=100,
                  interpolation="nearest")
        ax.set_facecolor("#585858")
    else:
        cmap = ListedColormap(["#5A5A5A", "#F2F2F2", "#101010"])
        ax.imshow(a, origin="lower", extent=extent, cmap=cmap,
                  norm=BoundaryNorm([-1.5, -0.5, 50, 101], cmap.N),
                  interpolation="nearest")
    # the car's path, drawn only on map-frame grids
    if track and msg.header.frame_id == "map":
        xs = [p[0] for p in track]; ys = [p[1] for p in track]
        ax.plot(xs, ys, "-", color="#FF3B30", linewidth=1.4, alpha=.9)
        ax.plot(xs[-1], ys[-1], "o", color="#FF3B30", markersize=5)
    unk = 100.0 * (a < 0).sum() / a.size
    ax.set_title(f"{title} — {w}x{h}, unknown {unk:.0f}%", fontsize=10)
    ax.set_xlabel("x [m]", fontsize=8); ax.set_ylabel("y [m]", fontsize=8)
    ax.tick_params(labelsize=7)


def main():
    os.makedirs(OUT, exist_ok=True)
    rclpy.init()
    n = Recorder()
    print(f"recording frames to {OUT} every {PERIOD:.1f}s — Ctrl-C to stop", flush=True)
    i = 0
    try:
        while rclpy.ok():
            t_end = time.time() + PERIOD
            while time.time() < t_end:
                rclpy.spin_once(n, timeout_sec=0.05)
            p = n.pose()
            if p is not None and (not n.track or
                                  abs(p[0]-n.track[-1][0]) + abs(p[1]-n.track[-1][1]) > 1e-3):
                n.track.append(p)
            if not n.grids:
                continue
            fig, axes = plt.subplots(1, 3, figsize=(16, 5.4), dpi=100)
            for ax, (topic, title, cm) in zip(axes, TOPICS):
                draw(ax, n.grids.get(topic), title, cm, n.track)
            fig.suptitle(f"F1TENTH exploration — frame {i}  |  t+{i*PERIOD:.0f}s",
                         fontsize=12)
            fig.tight_layout()
            fig.savefig(f"{OUT}/frame_{i:05d}.png", bbox_inches="tight")
            plt.close(fig)
            i += 1
            if i % 10 == 0:
                print(f"  {i} frames", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"stopped after {i} frames in {OUT}", flush=True)
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
