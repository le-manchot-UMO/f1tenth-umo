#!/usr/bin/env python3
"""How wide are the gaps, really? Answers: would a narrower car fit?

For a set of candidate half-widths, dilate the occupied cells by that radius and
count what free space survives, plus the largest connected free region. A car of
half-width r can only traverse cells that survive dilation by r.
"""
import time
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid

RADII_M = [0.10, 0.12, 0.14, 0.15, 0.18, 0.20, 0.25]

class G(Node):
    def __init__(self):
        super().__init__("gap_probe")
        self.msg = None
        self.create_subscription(OccupancyGrid, "/map", self.cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST))
    def cb(self, m):
        if self.msg is None: self.msg = m

def largest_component(mask):
    h, w = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    best = 0
    for sy in range(0, h, 3):
        for sx in range(0, w, 3):
            if not mask[sy, sx] or seen[sy, sx]: continue
            q = deque([(sy, sx)]); seen[sy, sx] = True; n = 0
            while q:
                y, x = q.popleft(); n += 1
                for dy, dx in ((1,0),(-1,0),(0,1),(0,-1)):
                    ny, nx = y+dy, x+dx
                    if 0 <= ny < h and 0 <= nx < w and mask[ny,nx] and not seen[ny,nx]:
                        seen[ny,nx] = True; q.append((ny,nx))
            best = max(best, n)
    return best

def main():
    rclpy.init(); n = G()
    end = time.time() + 25
    while time.time() < end and n.msg is None:
        rclpy.spin_once(n, timeout_sec=0.25)
    if n.msg is None: print("NO /map"); return
    m = n.msg; res = m.info.resolution
    a = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    occ = a > 50
    free = (a >= 0) & (a <= 20)
    print(f"map {m.info.width}x{m.info.height} @ {res:.3f} m/cell")
    print(f"free {free.sum()} cells = {free.sum()*res*res:.2f} m^2, "
          f"occupied {occ.sum()} cells\n")
    print(f"{'half-width':>11} {'traversable m^2':>16} {'% of free':>10} {'largest region m^2':>19}")
    for rm in RADII_M:
        r = int(round(rm / res))
        offs = [(dy,dx) for dy in range(-r,r+1) for dx in range(-r,r+1)
                if dy*dy+dx*dx <= r*r]
        dil = np.zeros_like(occ)
        for dy, dx in offs:
            dil |= np.roll(np.roll(occ, dy, 0), dx, 1)
        ok = free & ~dil
        comp = largest_component(ok) if ok.sum() else 0
        print(f"{rm:10.2f}m {ok.sum()*res*res:15.2f} {100*ok.sum()/max(free.sum(),1):9.1f}% "
              f"{comp*res*res:18.2f}")
    print("\nA car of half-width r can only drive where the r row is non-zero;")
    print("the largest-region column is the biggest area it can reach without teleporting.")
    n.destroy_node(); rclpy.shutdown()

main()
