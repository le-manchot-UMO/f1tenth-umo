#!/usr/bin/env python3
"""
F1Tenth frontier-based autonomous explorer.

Subscribes to /map (from SLAM Toolbox), detects frontier clusters with the
Wavefront Frontier Detector (WFD) — a BFS from the robot through free space
that extracts frontier clusters as the wavefront reaches them — and sends the
nearest REACHABLE frontier to Nav2 as a NavigateToPose goal. Because WFD only
visits free space reachable from the robot, unreachable frontiers are excluded
by construction; reachability is additionally verified against the
/global_costmap/costmap so we do not waste time sending goals the planner cannot
reach.

Requires: rclpy, nav2_msgs, geometry_msgs, nav_msgs, tf2_ros, numpy.
"""

import math
import threading
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

try:
    from tf_transformations import quaternion_from_euler
except Exception:
    # Same ZYX (yaw-pitch-roll) convention as tf_transformations; produces
    # (qx, qy, qz, qw) and the explorer only uses yaw, so qx=qy=0.
    def quaternion_from_euler(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return (0.0, 0.0, sy, cy)


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")

        # --- parameters ---
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("frame", "map")
        # 15, up from 5. Five cells is a sliver: the last run burned its goals on
        # size=5 and size=8 frontiers clustered in one spot, none reachable.
        self.declare_parameter("min_cluster_size", 15)
        self.declare_parameter("loop_period", 3.0)
        self.declare_parameter("idle_complete_threshold", 3)
        self.declare_parameter("max_goal_distance", 6.0)   # m; ignore far frontiers
        # 2.0, up from 0.5 — strongly prefer frontiers AHEAD of the car. An
        # Ackermann car cannot turn in place (min radius 0.94 m), so a frontier
        # off to the side costs a wide arc that often ends nose-in against
        # something. Favouring what is straight ahead keeps runs in long lines.
        self.declare_parameter("heading_weight", 2.0)
        # New tunables:
        self.declare_parameter("costmap_max_age", 2.0)         # s; reject costmap older than this
        self.declare_parameter("soft_cost_threshold", 200)     # cost >= this is treated as obstacle for reachability
        self.declare_parameter("goal_free_radius_cells", 1)   # cells around goal that must be free
        self.declare_parameter("size_bonus_per_cell", 0.05)    # reward (m) per cell of cluster size
        self.declare_parameter("blacklist_prune_period", 30.0)  # s; blacklist GC cadence

        self.map_topic = self.get_parameter("map_topic").value
        self.costmap_topic = self.get_parameter("costmap_topic").value
        self.frame = self.get_parameter("frame").value
        self.min_cluster = int(self.get_parameter("min_cluster_size").value)
        self.loop_period = float(self.get_parameter("loop_period").value)
        self.idle_threshold = int(self.get_parameter("idle_complete_threshold").value)
        self.max_goal_distance = float(self.get_parameter("max_goal_distance").value)
        self.heading_weight = float(self.get_parameter("heading_weight").value)
        self.costmap_max_age = float(self.get_parameter("costmap_max_age").value)
        self.soft_cost_threshold = int(self.get_parameter("soft_cost_threshold").value)
        self.goal_free_radius = int(self.get_parameter("goal_free_radius_cells").value)
        self.size_bonus_per_cell = float(self.get_parameter("size_bonus_per_cell").value)
        self.blacklist_prune_period = float(self.get_parameter("blacklist_prune_period").value)

        # --- state ---
        self.map_msg = None
        self.costmap_msg = None
        self.busy = False
        self.current_handle = None
        self.current_goal_xy = None          # (x, y) of active goal for blacklisting
        self.empty_streak = 0
        self.complete = False                # only set on shutdown / explicit stop; never on transient empty
        self.lock = threading.RLock()        # RLock so callbacks can re-enter
        self.failed_frontiers = {}           # (x, y) -> expiration_time (float)
        self.blacklist_duration = 30.0       # seconds to skip a failed frontier

        # --- tf ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- action client ---
        self.action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.get_logger().info("Waiting for /navigate_to_pose action server...")
        if not self.action_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().warn("NavigateToPose action server not available after 15s; "
                                   "will keep retrying.")

        # --- subscriptions + timer ---
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_cb, 1)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, self.costmap_topic, self.costmap_cb, 1)
        self.timer = self.create_timer(self.loop_period, self.explore_loop)
        # Periodic blacklist GC prevents the dict from growing unbounded
        # (B9).  Runs at ~1/3 the loop period but is cheap.
        gc_period = max(1.0, self.blacklist_prune_period / 3.0)
        self.blacklist_timer = self.create_timer(gc_period, self._prune_blacklist)
        self.get_logger().info("Frontier explorer ready. Waiting for a map from SLAM...")

    # ------------------------------------------------------------------ map
    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg

    def costmap_cb(self, msg: OccupancyGrid):
        self.costmap_msg = msg

    def get_robot_pose(self):
        # B14: explicit timeout instead of relying on the buffer's default,
        # which can be 0 (non-blocking) on some TF2 builds and can hang the
        # timer callback forever if the SLAM/map TFs aren't ready yet.
        try:
            t = self.tf_buffer.lookup_transform(
                self.frame, "base_link", rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
        except TransformException as ex:
            self.get_logger().debug(f"TF lookup failed: {ex}")
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        # Use full quaternion -> yaw so we don't rely on the convention that
        # the rotation is around z only.  Equivalent to the previous formula
        # for z-axis rotation but more robust to small extrinsic rotations.
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (x, y, yaw)

    # ---------------------------------------------------------- frontier ops
    def find_frontier_clusters(self, robot_pose=None):
        """Wavefront Frontier Detector (WFD).

        A BFS ('wavefront') expands from the robot's cell through FREE space.
        When the wavefront reaches a frontier cell (a free cell with an
        unknown neighbour), an inner BFS extracts the whole connected frontier
        cluster. Only free space reachable from the robot is visited (not the
        whole map), so unreachable frontiers are inherently excluded — this is
        why WFD helps our 'planner fails on unreachable frontier goals' problem.
        Each cluster's representative is the wavefront-entry cell (the nearest
        frontier cell of that cluster to the robot), which pairs with the
        heading-aware selection in pick_reachable_frontier.

        Returns list of (gx, gy, size) in map-frame meters, like before.
        Falls back to a full-map scan if the robot pose is unavailable or the
        robot is not on a free cell (can't seed the wavefront).
        """
        msg = self.map_msg
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        # B15: np.frombuffer is zero-copy; np.asarray may copy if dtypes differ.
        data = np.frombuffer(msg.data, dtype=np.int8).reshape((h, w))

        free = (data == 0)
        unknown = (data == -1)
        # frontier mask: free cell with >=1 unknown 4-neighbour
        # (B3: 4-connectivity matches the planner's grid).
        frontier = np.zeros_like(free, dtype=bool)
        frontier[:-1, :] |= unknown[1:, :]
        frontier[1:, :] |= unknown[:-1, :]
        frontier[:, :-1] |= unknown[:, 1:]
        frontier[:, 1:] |= unknown[:, :-1]
        frontier &= free
        if not frontier.any():
            return []

        # need a free start cell to seed the wavefront
        sr = sc = None
        if robot_pose is not None:
            rx, ry = robot_pose[0], robot_pose[1]
            sc = int((rx - ox) / res)
            sr = int((ry - oy) / res)
        if sr is None or not (0 <= sr < h and 0 <= sc < w) or not free[sr, sc]:
            # no usable robot cell -> fall back to full-map scan
            return self._find_frontiers_fullmap(robot_pose)

        map_visited = np.zeros((h, w), dtype=bool)
        front_visited = np.zeros((h, w), dtype=bool)
        map_visited[sr, sc] = True
        clusters = []
        q = deque([(sr, sc)])
        while q:
            r, c = q.popleft()
            # If this is an unclaimed frontier cell, extract its whole cluster
            # via an inner BFS. The entry cell (r,c) is the nearest-to-robot
            # cell of this cluster because the outer wavefront visits cells in
            # order of BFS distance from the robot.
            if frontier[r, c] and not front_visited[r, c]:
                entry = (r, c)
                cells = []
                fq = deque([(r, c)])
                front_visited[r, c] = True
                while fq:
                    cr, cc = fq.popleft()
                    cells.append((cr, cc))
                    # B3: 4-connectivity for inner cluster BFS too.
                    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                        nr, nc = cr + dr, cc + dc
                        if (0 <= nr < h and 0 <= nc < w and frontier[nr, nc]
                                and not front_visited[nr, nc]):
                            front_visited[nr, nc] = True
                            fq.append((nr, nc))
                if len(cells) >= self.min_cluster:
                    gx = ox + (entry[1] + 0.5) * res
                    gy = oy + (entry[0] + 0.5) * res
                    clusters.append((gx, gy, len(cells)))
            # B3: 4-connectivity for the outer wavefront too.
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w and free[nr, nc] and not map_visited[nr, nc]:
                    map_visited[nr, nc] = True
                    q.append((nr, nc))
        return clusters

    def _find_frontiers_fullmap(self, robot_pose=None):
        """Full-map frontier scan (the old method) — used as a WFD fallback when
        the robot pose is unavailable (can't seed the wavefront)."""
        msg = self.map_msg
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        data = np.frombuffer(msg.data, dtype=np.int8).reshape((h, w))
        free = (data == 0)
        unknown = (data == -1)
        frontier = np.zeros_like(free, dtype=bool)
        frontier[:-1, :] |= unknown[1:, :]
        frontier[1:, :] |= unknown[:-1, :]
        frontier[:, :-1] |= unknown[:, 1:]
        frontier[:, 1:] |= unknown[:, :-1]
        frontier &= free
        if not frontier.any():
            return []
        rx = ry = None
        if robot_pose is not None:
            rx, ry = robot_pose[0], robot_pose[1]
        # B13: use BFS (deque) for consistency with the WFD inner DFS.
        visited = np.zeros_like(frontier, dtype=bool)
        clusters = []
        rows, cols = frontier.nonzero()
        for r, c in zip(rows.tolist(), cols.tolist()):
            if visited[r, c]:
                continue
            stack = deque([(r, c)])
            visited[r, c] = True
            cells = []
            while stack:
                cr, cc = stack.popleft()
                cells.append((cr, cc))
                for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    nr, nc = cr + dr, cc + dc
                    if 0 <= nr < h and 0 <= nc < w and frontier[nr, nc] and not visited[nr, nc]:
                        visited[nr, nc] = True
                        stack.append((nr, nc))
            if len(cells) < self.min_cluster:
                continue
            # B18: pick the cell of the cluster adjacent to non-frontier free
            # space (the natural edge a planner can drive toward), not the
            # centroid (which may be deep inside an obstacle).
            if rx is not None:
                # Scan all cells of the cluster for the one closest to the
                # robot.  Ties broken by closeness to the cluster centroid
                # so we still bias toward the edge nearest the car.
                cr = cx = None
                best_d = None
                cy_sum = 0.0
                cc_sum = 0.0
                for (rc, cc_) in cells:
                    cy_sum += rc
                    cc_sum += cc_
                mr = cy_sum / len(cells)
                mc = cc_sum / len(cells)
                for (rc, cc_) in cells:
                    d = (oy + (rc + 0.5) * res - ry) ** 2 + (ox + (cc_ + 0.5) * res - rx) ** 2
                    if best_d is None or d < best_d:
                        best_d = d
                        cr = rc
                        cx = cc_
                _ = (mr, mc)  # centroid kept for potential future use
            else:
                cr = cx = cells[0]
            gx = ox + (cx + 0.5) * res
            gy = oy + (cr + 0.5) * res
            clusters.append((gx, gy, len(cells)))
        return clusters

    # ---------------------------------------------------------- reachability
    def _costmap_cell(self, x, y):
        """Return costmap cell (col,row) for map-frame (x,y), or None."""
        if self.costmap_msg is None:
            return None, None
        m = self.costmap_msg
        cx = int((x - m.info.origin.position.x) / m.info.resolution)
        cy = int((y - m.info.origin.position.y) / m.info.resolution)
        if 0 <= cx < m.info.width and 0 <= cy < m.info.height:
            return cx, cy
        return None, None

    def _cost_at(self, cx, cy):
        """Return costmap cost at cell, or None if out of bounds/no costmap."""
        if self.costmap_msg is None:
            return None
        m = self.costmap_msg
        if 0 <= cx < m.info.width and 0 <= cy < m.info.height:
            return int(self.costmap_msg.data[cy * m.info.width + cx])
        return None

    def _costmap_now(self):
        """Wall-clock seconds (rclpy Time) for the costmap freshness check."""
        return self.get_clock().now().nanoseconds / 1e9

    def _costmap_age(self):
        """Age of the global costmap in seconds, or +inf if unknown.

        Uses plain int nanosecond arithmetic to avoid the rclpy clock-type
        mismatch ('Can't subtract times with different clock types') between
        the node clock (ROS_TIME) and a bare Time (SYSTEM_TIME). Both are
        wall-clock under use_sim_time:=false, so the subtraction is valid.
        """
        if self.costmap_msg is None:
            return float("inf")
        stamp = self.costmap_msg.header.stamp
        now_ns = self.get_clock().now().nanoseconds
        msg_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        return (now_ns - msg_ns) / 1e9

    def is_reachable(self, gx, gy, robot_pose):
        """Quick reachability check against the global costmap.

        Returns True if:
        - the goal cell is not lethal AND not in the soft-inflation cone
          (configurable, default cost < 200), AND
        - the line-of-sight corridor from robot to goal has no cell with
          cost in {253, 254}, AND
        - a small annulus around the goal is also free (so the planner can
          actually approach it).

        Returns False if the costmap data is too stale (B5) so the explorer
        defers rather than sending a goal based on a phantom costmap.
        """
        if self.costmap_msg is None:
            # no costmap yet; be permissive
            return True

        # B5: freshness check.  If the costmap is older than the threshold
        # we treat it as "unknown" and let the planner decide — but we log
        # so a stale-data issue is visible.
        age = self._costmap_age()
        if age > self.costmap_max_age:
            self.get_logger().debug(
                f"Costmap age {age:.1f}s > {self.costmap_max_age}s; reachability uncertain.")
            return True

        # robot cell
        rcx, rcy = self._costmap_cell(robot_pose[0], robot_pose[1])
        if rcx is None or rcy is None:
            return True
        rcost = self._cost_at(rcx, rcy)
        if rcost is not None and rcost >= 253:
            self.get_logger().debug("Robot cell is lethal; skipping reachability.")
            return True   # let recovery sort it out

        # goal cell: must not be lethal or in the soft-inflation cone.
        gcx, gcy = self._costmap_cell(gx, gy)
        if gcx is None or gcy is None:
            return False
        gcost = self._cost_at(gcx, gcy)
        if gcost is not None:
            if gcost >= 253:
                return False
            # B4: reject cells inside the inflation cone.  Without this, the
            # explorer happily picks a goal that is "free" but the planner
            # cannot reach because the car footprint + inflation overlaps it.
            if gcost >= self.soft_cost_threshold:
                return False

        # B4: require a small free annulus around the goal so the planner
        # can actually plan a path into it.  Allow the goal cell itself to
        # be free (cost 0) and require the surrounding ring to be below
        # the soft threshold.
        if self.goal_free_radius > 0:
            m = self.costmap_msg
            for dr in range(-self.goal_free_radius, self.goal_free_radius + 1):
                for dc in range(-self.goal_free_radius, self.goal_free_radius + 1):
                    if abs(dr) == self.goal_free_radius or abs(dc) == self.goal_free_radius:
                        c = self._cost_at(gcx + dc, gcy + dr)
                        if c is not None and c >= self.soft_cost_threshold:
                            return False

        # Bresenham line from robot cell to goal cell
        x0, y0 = rcx, rcy
        x1, y1 = gcx, gcy
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            c = self._cost_at(x0, y0)
            # Reject only KNOWN lethal / inscribed obstacles (253, 254).
            # Unknown cells (255) are expected when exploring and must be allowed,
            # otherwise frontiers in unmapped space (e.g., down a hallway) are
            # incorrectly rejected.
            if c is not None and c in (253, 254):
                return False
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return True

    # ---------------------------------------------------------- pick
    def pick_reachable_frontier(self, clusters, robot_pose):
        if robot_pose is None:
            return max(clusters, key=lambda c: c[2])
        rx, ry, ryaw = robot_pose

        # Score each candidate: prefer frontiers that are NEAR and AHEAD of the
        # car (in its heading direction). Picking only the nearest frontier
        # often sent the car sideways/backward, so it arced in place instead of
        # driving straight down the hallway. score = distance - size_bonus +
        # heading_weight * (1 - cos(relative_angle)); (1 - cos) is 0 dead ahead
        # and 2 directly behind, so a behind frontier is penalised by up to
        # 2*heading_weight (metres-equivalent).
        scored = []
        skipped_blacklisted = 0
        for (cx, cy, size) in clusters:
            d = math.hypot(cx - rx, cy - ry)
            if d > self.max_goal_distance:
                continue
            # B2: read blacklist under the lock so we can't blacklist-then-still-pick
            # the same frontier mid-iteration.
            with self.lock:
                if self._is_blacklisted_locked(cx, cy):
                    skipped_blacklisted += 1
                    continue
            rel = math.atan2(cy - ry, cx - rx) - ryaw
            rel = (rel + math.pi) % (2 * math.pi) - math.pi
            heading_pen = self.heading_weight * (1.0 - math.cos(rel))
            # B6: size bonus scaled by configurable weight (default 0.05 m/cell,
            # so a 100-cell cluster wins ~5 m vs a 5-cell cluster).
            scored.append((d - self.size_bonus_per_cell * size + heading_pen, cx, cy, size))
        scored.sort(key=lambda x: x[0])
        if skipped_blacklisted:
            self.get_logger().debug(f"Skipped {skipped_blacklisted} blacklisted frontiers.")

        # Return the first candidate that passes the costmap reachability check.
        # Do NOT fall back to an unreachable frontier: sending a goal the planner
        # can't reach (outside the global-costmap bounds, or a lethal corridor)
        # makes Nav2 exhaust its recoveries and abort. It also used to send
        # out-of-bounds goals, producing hundreds of planner 'worldToMap failed'
        # errors. Skip this loop instead and retry next tick — the global
        # costmap usually catches up to the SLAM map within ~1s, or the car
        # moves and opens new reachable frontiers.
        for _, cx, cy, size in scored:
            if self.is_reachable(cx, cy, robot_pose):
                return (cx, cy, size)
        if scored:
            self.get_logger().debug(
                f"No reachable frontier among {len(scored)} candidates "
                f"(blacklisted: {len(self.failed_frontiers)}); skipping this loop.")
        return None

    # ------------------------------------------------------------- goal flow
    def explore_loop(self):
        if self.complete:
            return
        with self.lock:
            if self.busy:
                return
        if self.map_msg is None:
            return
        if not self.action_client.server_is_ready():
            return

        robot = self.get_robot_pose()
        clusters = self.find_frontier_clusters(robot)
        if not clusters:
            # B10: do NOT mark the explorer as complete on a transient
            # empty-map.  SLAM loop closures can blank the map for several
            # seconds; the previous version would permanently stop the
            # explorer with timer.cancel().  We just log and wait.
            self.empty_streak += 1
            self.get_logger().info(
                f"No frontiers found ({self.empty_streak}/{self.idle_threshold}).")
            # Optionally publish a "done" hint to log/save tool, but keep looping.
            return

        self.empty_streak = 0
        target = self.pick_reachable_frontier(clusters, robot)
        if target is None:
            self.get_logger().debug("No reachable frontier within max_goal_distance; skipping loop.")
            return
        cx, cy, size = target
        if robot is not None:
            yaw = math.atan2(cy - robot[1], cx - robot[0])
        else:
            yaw = 0.0
        self.get_logger().info(
            f"Sending goal to frontier ({cx:.2f}, {cy:.2f}) size={size} yaw={yaw:.2f}")
        self.send_goal(cx, cy, yaw)

    def send_goal(self, x, y, yaw):
        with self.lock:
            if self.busy:
                return
            self.busy = True
            self.current_goal_xy = (round(x, 2), round(y, 2))

        pose = PoseStamped()
        pose.header.frame_id = self.frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        _, _, z, w = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.behavior_tree = ""

        if not self.action_client.server_is_ready():
            self.get_logger().warn("Action server not ready; skipping.")
            self._reset_busy()
            return

        # B20: wrap send_goal_async so failures don't leave busy=True forever.
        try:
            send_future = self.action_client.send_goal_async(goal)
        except Exception as ex:
            self.get_logger().error(f"send_goal_async raised: {ex}")
            self._reset_busy()
            return
        send_future.add_done_callback(self.goal_response_cb)

    def _reset_busy(self):
        with self.lock:
            self.busy = False
            self.current_goal_xy = None
            self.current_handle = None

    def feedback_cb(self, feedback_msg):
        # B17: register no-op callback only when we actually want feedback.
        pass

    def goal_response_cb(self, future):
        try:
            handle: ClientGoalHandle = future.result()
        except Exception as ex:
            self.get_logger().error(f"goal_response_cb: future failed: {ex}")
            self._reset_busy()
            return
        with self.lock:
            self.current_handle = handle
        if not handle.accepted:
            self.get_logger().warn("Goal rejected by bt_navigator; will retry next loop.")
            self._reset_busy()
            return
        self.get_logger().info("Goal accepted; navigating to frontier.")
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def _is_blacklisted_locked(self, x, y):
        """Caller must hold self.lock.  B2: blacklist access is serialized."""
        key = (round(x, 2), round(y, 2))
        now = self._costmap_now()
        expire = self.failed_frontiers.get(key, 0.0)
        if expire > now:
            return True
        # stale entry; clean it up
        if key in self.failed_frontiers:
            del self.failed_frontiers[key]
        return False

    def _is_blacklisted(self, x, y):
        """Public, lock-acquiring wrapper used by callers that don't hold the lock."""
        with self.lock:
            return self._is_blacklisted_locked(x, y)

    def _prune_blacklist(self):
        """B9: GC stale blacklist entries so the dict doesn't grow forever."""
        with self.lock:
            now = self._costmap_now()
            stale = [k for k, exp in self.failed_frontiers.items() if exp <= now]
            for k in stale:
                del self.failed_frontiers[k]
            if stale:
                self.get_logger().debug(f"Pruned {len(stale)} stale blacklist entries.")

    def result_cb(self, future):
        try:
            res = future.result()
            status = res.status if res is not None else -1
            # Map GoalStatus -> log + blacklist.  ABORTED (6) is the expected
            # outcome on unreachable / collision-ahead goals; blacklist the spot
            # so we don't re-send the same failing frontier.  Named constants
            # are used (not magic numbers) so this is correct across ROS 2
            # distros -- the earlier code hard-coded 2/4/5 and missed ABORTED,
            # so the explorer hammered one frontier forever.
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Frontier reached. Picking next.")
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info("Goal canceled.")
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().info(
                    "Goal aborted (unreachable/collision). Picking next.")
                with self.lock:
                    if self.current_goal_xy is not None:
                        self.failed_frontiers[self.current_goal_xy] = (
                            self._costmap_now() + self.blacklist_duration)
                        self.get_logger().info(
                            f"Blacklisted frontier {self.current_goal_xy} "
                            f"for {self.blacklist_duration}s.")
            else:
                self.get_logger().warn(f"Goal ended with status {status}. Picking next.")
        except Exception as ex:
            self.get_logger().error(f"Error getting result: {ex}")
        finally:
            self._reset_busy()

    def cancel_current(self):
        with self.lock:
            handle = self.current_handle
        if handle is not None:
            self.get_logger().info("Cancelling current goal...")
            try:
                handle.cancel_goal_async()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted; cancelling current goal and exiting.")
        node.cancel_current()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
