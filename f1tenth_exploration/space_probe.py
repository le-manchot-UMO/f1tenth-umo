#!/usr/bin/env python3
"""How boxed in is the car, really? Reports clearance vs costmap cost."""
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf2_ros

class P(Node):
    def __init__(self):
        super().__init__("space_probe")
        self.scan=None; self.cm=None
        self.buf=tf2_ros.Buffer(); tf2_ros.TransformListener(self.buf,self)
        self.create_subscription(LaserScan,"/scan",self.s_cb,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST))
        self.create_subscription(OccupancyGrid,"/local_costmap/costmap",self.c_cb,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,history=HistoryPolicy.KEEP_LAST))
    def s_cb(self,m): self.scan=m
    def c_cb(self,m): self.cm=m

def main():
    rclpy.init(); n=P()
    end=time.time()+20
    while time.time()<end and (n.scan is None or n.cm is None):
        rclpy.spin_once(n,timeout_sec=0.2)
    if n.scan is None: print("NO SCAN"); return
    s=n.scan
    # clearance in the four sectors that matter
    def sector(lo,hi):
        best=(999,0)
        for i,r in enumerate(s.ranges):
            a=s.angle_min+i*s.angle_increment
            if lo<=a<=hi and s.range_min<r<s.range_max and math.isfinite(r):
                if r<best[0]: best=(r,a)
        return best
    for name,lo,hi in [("FRONT +-20deg",-0.35,0.35),("LEFT 60-120",1.05,2.09),
                       ("RIGHT -120--60",-2.09,-1.05),("REAR |a|>150",2.62,3.15)]:
        r,a=sector(lo,hi)
        print(f"{name:16s} nearest={r:.2f} m at {math.degrees(a):+.0f} deg")
    r,a=sector(-3.15,3.15); print(f"{'ANY':16s} nearest={r:.2f} m at {math.degrees(a):+.0f} deg")

    if n.cm is None: print("NO COSTMAP"); return
    c=n.cm; info=c.info
    try:
        t=n.buf.lookup_transform(info.header.frame_id if info.header.frame_id else "odom",
                                 "base_link", rclpy.time.Time())
    except Exception as e:
        try: t=n.buf.lookup_transform("odom","base_link",rclpy.time.Time())
        except Exception as e2: print("no TF:",e2); return
    rx=t.transform.translation.x; ry=t.transform.translation.y
    q=t.transform.rotation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    print(f"robot in {c.header.frame_id}: x={rx:.2f} y={ry:.2f} yaw={math.degrees(yaw):.0f}deg")
    def cost(px,py):
        cx=int((px-info.origin.position.x)/info.resolution)
        cy=int((py-info.origin.position.y)/info.resolution)
        if 0<=cx<info.width and 0<=cy<info.height: return c.data[cy*info.width+cx]
        return None
    print(f"cost at robot centre: {cost(rx,ry)}")
    # cost along the car's own footprint corners
    for lbl,dx,dy in [("front-L",0.35,0.20),("front-R",0.35,-0.20),
                      ("rear-L",-0.35,0.20),("rear-R",-0.35,-0.20)]:
        px=rx+dx*math.cos(yaw)-dy*math.sin(yaw); py=ry+dx*math.sin(yaw)+dy*math.cos(yaw)
        print(f"  footprint {lbl:8s} cost={cost(px,py)}")
    # cost straight ahead / behind
    for d in (0.3,0.5,0.8,1.2,-0.3,-0.5):
        px=rx+d*math.cos(yaw); py=ry+d*math.sin(yaw)
        print(f"  {d:+.1f} m along heading cost={cost(px,py)}")
    n.destroy_node(); rclpy.shutdown()
main()
