#!/usr/bin/env python3
"""
path_planner_node.py

Computes a collision-free lap path through the SLAM map using A*.

Because the goal is the same as the start (closed loop), a single A* call
would find the trivial zero-length path.  Instead, the planner:
  1. Loads the lap-1 recorded path (saved by path_recorder_node).
  2. Samples `num_via_points` evenly-spaced waypoints from it as via-points.
  3. Chains A* segments: start → wp1 → wp2 → ... → wpN → start.
  4. Each segment is obstacle-checked against the inflated SLAM map.
  5. Publishes the concatenated path on /planned_path (nav_msgs/Path).

This satisfies "path computed from the map" (A* respects the occupancy grid)
while guaranteeing the robot covers the full closed-loop course.

Parameters:
  path_file        (str,   default ~/.ros/recorded_path.yaml)
  num_via_points   (int,   default 6)    — waypoints sampled from recorded path
  inflation_radius (float, default 0.30) — obstacle clearance in metres
"""

import heapq
import math
import os
import yaml

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import tf2_ros

_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

OBSTACLE_THRESH = 50


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__("path_planner_node")

        self.declare_parameter("path_file",
                               os.path.expanduser("~/.ros/recorded_path.yaml"))
        self.declare_parameter("num_via_points", 6)
        self.declare_parameter("inflation_radius", 0.30)

        self._path_file   = self.get_parameter("path_file").value
        self._num_via     = self.get_parameter("num_via_points").value
        self._infl_r      = self.get_parameter("inflation_radius").value

        self._map: OccupancyGrid | None = None
        self._racing = False

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._path_pub     = self.create_publisher(Path,          "/planned_path",  _LATCHED_QOS)
        self._inflated_pub = self.create_publisher(OccupancyGrid, "/inflated_map",  _LATCHED_QOS)
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, 1)
        self.create_subscription(String, "/slam_coordinator/mode", self._mode_cb, 10)

        self.get_logger().info(
            f"Path planner ready — via_points={self._num_via}, "
            f"inflation={self._infl_r:.2f} m. Waiting for racing mode."
        )

    # ------------------------------------------------------------------

    def _map_cb(self, msg: OccupancyGrid):
        self._map = msg

    def _mode_cb(self, msg: String):
        if msg.data in ("ready", "racing") and not self._racing:
            self._racing = True
            self.get_logger().info(f"{msg.data.upper()} mode — planning lap path...")
            self._plan()

    # ------------------------------------------------------------------
    # Top-level plan
    # ------------------------------------------------------------------

    def _plan(self):
        if self._map is None:
            self.get_logger().error("No map received — cannot plan")
            return

        # Build inflated obstacle grid for dashboard visualisation
        info   = self._map.info
        res    = info.resolution
        w      = info.width
        h      = info.height
        ox     = info.origin.position.x
        oy     = info.origin.position.y

        grid_data = np.array(self._map.data, dtype=np.int16).reshape(h, w)
        obstacle  = (grid_data >= OBSTACLE_THRESH) | (grid_data < 0)

        infl_cells = max(1, int(math.ceil(self._infl_r / res)))
        from scipy.ndimage import binary_dilation
        _r = infl_cells
        _y, _x = np.ogrid[-_r:_r + 1, -_r:_r + 1]
        struct   = (_x ** 2 + _y ** 2) <= _r ** 2
        inflated = binary_dilation(obstacle, structure=struct)

        inflated_msg = OccupancyGrid()
        inflated_msg.header.stamp    = self.get_clock().now().to_msg()
        inflated_msg.header.frame_id = "map"
        inflated_msg.info            = self._map.info
        inflated_msg.data            = [100 if v else 0 for v in inflated.flatten().tolist()]
        self._inflated_pub.publish(inflated_msg)

        # Use recorded path directly as planned path
        try:
            with open(self._path_file) as f:
                data = yaml.safe_load(f)
            poses = data["poses"]
        except Exception as e:
            self.get_logger().error(f"Failed to load recorded path: {e}")
            return

        if not poses:
            self.get_logger().error("Recorded path is empty")
            return

        path_msg = Path()
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for p in poses:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x    = p["x"]
            ps.pose.position.y    = p["y"]
            ps.pose.orientation.z = p.get("qz", 0.0)
            ps.pose.orientation.w = p.get("qw", 1.0)
            path_msg.poses.append(ps)

        self._path_pub.publish(path_msg)
        self.get_logger().info(
            f"Published planned path: {len(path_msg.poses)} waypoints (recorded path)"
        )

    # ------------------------------------------------------------------
    # Sample evenly-spaced via-points from the recorded path
    # ------------------------------------------------------------------

    def _sample_via_points(
        self, start_wx: float, start_wy: float
    ) -> list[tuple[float, float]] | None:
        try:
            with open(self._path_file) as f:
                data = yaml.safe_load(f)
            poses = data["poses"]
        except Exception as e:
            self.get_logger().error(f"Failed to load recorded path: {e}")
            return None

        if len(poses) < self._num_via + 1:
            self.get_logger().warn(
                f"Recorded path only has {len(poses)} poses — using all"
            )
            return [(p["x"], p["y"]) for p in poses]

        # ------------------------------------------------------------------
        # Filter to "balanced" (straight) portions only.
        #
        # Compute curvature = |dθ/ds| at each pose:
        #   θ   from stored qz/qw (yaw = 2*atan2(qz, qw))
        #   ds  from Euclidean distance to next pose
        #
        # A sliding window (±WINDOW poses) smooths noise. Poses whose
        # smoothed curvature exceeds CURVATURE_THRESH rad/m are corners and
        # are excluded from via-point candidacy.
        # ------------------------------------------------------------------
        CURVATURE_THRESH = 0.35   # rad/m — below this = straight/balanced
        WINDOW           = 5      # half-width of smoothing window

        n = len(poses)
        yaws = [2.0 * math.atan2(p.get("qz", 0.0), p.get("qw", 1.0))
                for p in poses]

        # Arc-length between consecutive poses
        dists = [
            math.hypot(poses[i+1]["x"] - poses[i]["x"],
                       poses[i+1]["y"] - poses[i]["y"])
            for i in range(n - 1)
        ]

        # Instantaneous |dθ/ds| (guard against zero ds)
        def wrap(a):
            return math.atan2(math.sin(a), math.cos(a))

        raw_curv = [
            abs(wrap(yaws[i+1] - yaws[i])) / max(dists[i], 0.01)
            for i in range(n - 1)
        ]
        raw_curv.append(raw_curv[-1])  # pad to length n

        # Smooth curvature with a sliding window max (conservative: a pose is
        # "in a corner" if any nearby pose has high curvature)
        smoothed = []
        for i in range(n):
            lo = max(0, i - WINDOW)
            hi = min(n, i + WINDOW + 1)
            smoothed.append(max(raw_curv[lo:hi]))

        straight_idxs = [
            i for i, c in enumerate(smoothed)
            if c < CURVATURE_THRESH
            and i > 0               # skip very start
            and i < n - 1           # skip very end
        ]

        self.get_logger().info(
            f"Recorded path: {n} poses, {len(straight_idxs)} in straight sections "
            f"(curvature < {CURVATURE_THRESH:.2f} rad/m)"
        )

        if len(straight_idxs) < self._num_via:
            self.get_logger().warn(
                "Not enough straight poses for requested via-points — "
                "lowering threshold and using all straight poses"
            )
            return [(poses[i]["x"], poses[i]["y"]) for i in straight_idxs]

        # Sample num_via evenly from the straight candidates (by position in
        # the list, not by index, so spacing is even around the course)
        step = len(straight_idxs) / (self._num_via + 1)
        sel  = [straight_idxs[int(round((k + 1) * step))]
                for k in range(self._num_via)]
        sel  = [min(i, len(straight_idxs) - 1) for i in sel]
        via  = [(poses[i]["x"], poses[i]["y"]) for i in sel]

        self.get_logger().info(
            f"Sampled {len(via)} via-points from straight sections "
            f"(pose indices: {sel})"
        )
        return via

    # ------------------------------------------------------------------
    # A* search
    # ------------------------------------------------------------------

    def _astar(
        self,
        obstacle: np.ndarray,
        start: tuple[int, int],
        goal: tuple[int, int],
        h: int,
        w: int,
    ) -> list[tuple[int, int]] | None:
        sr, sc = start
        gr, gc = goal

        def heuristic(r, c):
            return math.hypot(r - gr, c - gc)

        open_heap: list = []
        heapq.heappush(open_heap, (heuristic(sr, sc), 0.0, sr, sc))

        came_from: dict[tuple, tuple | None] = {(sr, sc): None}
        g_score:   dict[tuple, float]        = {(sr, sc): 0.0}
        closed:    set                       = set()

        DIRS = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

        while open_heap:
            f, g, r, c = heapq.heappop(open_heap)

            if (r, c) in closed:
                continue
            closed.add((r, c))

            if (r, c) == (gr, gc):
                path: list[tuple[int, int]] = []
                node: tuple[int, int] | None = (gr, gc)
                while node is not None:
                    path.append(node)
                    node = came_from[node]
                path.reverse()
                return path

            for i, (dr, dc) in enumerate(DIRS):
                nr, nc = r + dr, c + dc
                cost   = 1.0 if i < 4 else math.sqrt(2)
                if not (0 <= nr < h and 0 <= nc < w):
                    continue
                if obstacle[nr, nc]:
                    continue
                if (nr, nc) in closed:
                    continue
                ng = g + cost
                if ng < g_score.get((nr, nc), float("inf")):
                    g_score[(nr, nc)]  = ng
                    came_from[(nr, nc)] = (r, c)
                    heapq.heappush(open_heap,
                                   (ng + heuristic(nr, nc), ng, nr, nc))

        return None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _nearest_free(
        self,
        cell: tuple[int, int],
        obstacle: np.ndarray,
        h: int,
        w: int,
        max_search: int = 50,
    ) -> tuple[int, int] | None:
        """BFS outward from cell to find nearest non-obstacle cell."""
        if not obstacle[cell[0], cell[1]]:
            return cell
        from collections import deque
        q: deque = deque([cell])
        visited = {cell}
        while q:
            r, c = q.popleft()
            for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                nr, nc = r + dr, c + dc
                if (nr, nc) in visited:
                    continue
                if not (0 <= nr < h and 0 <= nc < w):
                    continue
                if abs(nr - cell[0]) + abs(nc - cell[1]) > max_search:
                    continue
                visited.add((nr, nc))
                if not obstacle[nr, nc]:
                    return (nr, nc)
                q.append((nr, nc))
        return None

    def _prune(
        self,
        path: list[tuple[int, int]],
        obstacle: np.ndarray,
        w: int,
        h: int,
    ) -> list[tuple[int, int]]:
        if len(path) <= 2:
            return path
        pruned = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if self._los_clear(path[i], path[j], obstacle, w, h):
                    break
                j -= 1
            pruned.append(path[j])
            i = j
        return pruned

    def _los_clear(
        self,
        a: tuple[int, int],
        b: tuple[int, int],
        obstacle: np.ndarray,
        w: int,
        h: int,
    ) -> bool:
        r0, c0 = a
        r1, c1 = b
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        r, c = r0, c0
        sr = 1 if r1 > r0 else -1
        sc = 1 if c1 > c0 else -1
        err = dr - dc
        while True:
            if not (0 <= r < obstacle.shape[0] and 0 <= c < w):
                return False
            if obstacle[r, c]:
                return False
            if r == r1 and c == c1:
                return True
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r   += sr
            if e2 < dr:
                err += dr
                c   += sc


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
