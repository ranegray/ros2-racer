#!/usr/bin/env python3
"""
path_planner_node.py

Computes a collision-free path through the SLAM map using A*.

Workflow:
  1. Receives the occupancy grid from /map (built by slam_toolbox in lap 1).
  2. Inflates obstacles by inflation_radius to give the robot body clearance.
  3. On mode switch to "racing", gets current robot pose from TF (map→base_link)
     and plans A* from that start to (goal_x, goal_y) in the map frame.
  4. Prunes collinear waypoints (line-of-sight check) to smooth the path.
  5. Publishes the result on /planned_path (nav_msgs/Path).

Parameters:
  goal_x           (float, default 0.0)  — goal x in map frame (metres)
  goal_y           (float, default 0.0)  — goal y in map frame (metres)
  inflation_radius (float, default 0.30) — obstacle inflation in metres
"""

import heapq
import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import tf2_ros


# Cells with occupancy >= this are treated as obstacles
OBSTACLE_THRESH = 50


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__("path_planner_node")

        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("inflation_radius", 0.30)

        self._goal_x  = self.get_parameter("goal_x").value
        self._goal_y  = self.get_parameter("goal_y").value
        self._infl_r  = self.get_parameter("inflation_radius").value

        self._map: OccupancyGrid | None = None
        self._racing = False

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._path_pub = self.create_publisher(Path, "/planned_path", 1)

        self.create_subscription(OccupancyGrid, "/map", self._map_cb, 1)
        self.create_subscription(String, "/slam_coordinator/mode", self._mode_cb, 10)

        self.get_logger().info(
            f"Path planner ready — goal=({self._goal_x:.2f}, {self._goal_y:.2f}) m, "
            f"inflation={self._infl_r:.2f} m. Waiting for racing mode."
        )

    # ------------------------------------------------------------------

    def _map_cb(self, msg: OccupancyGrid):
        self._map = msg

    def _mode_cb(self, msg: String):
        if msg.data == "racing" and not self._racing:
            self._racing = True
            self.get_logger().info("Racing mode — planning path...")
            self._plan()

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def _plan(self):
        if self._map is None:
            self.get_logger().error("No map received yet — cannot plan")
            return

        # Get current robot pose in map frame
        try:
            tf = self._tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return

        start_wx = tf.transform.translation.x
        start_wy = tf.transform.translation.y

        info = self._map.info
        res  = info.resolution
        w    = info.width
        h    = info.height
        ox   = info.origin.position.x
        oy   = info.origin.position.y

        def world_to_grid(wx, wy):
            col = int((wx - ox) / res)
            row = int((wy - oy) / res)
            return row, col

        def grid_to_world(row, col):
            wx = ox + (col + 0.5) * res
            wy = oy + (row + 0.5) * res
            return wx, wy

        start = world_to_grid(start_wx, start_wy)
        goal  = world_to_grid(self._goal_x, self._goal_y)

        # Build inflated obstacle grid
        grid_data = np.array(self._map.data, dtype=np.int16).reshape(h, w)
        obstacle = (grid_data >= OBSTACLE_THRESH)

        infl_cells = max(1, int(math.ceil(self._infl_r / res)))
        from scipy.ndimage import binary_dilation  # type: ignore
        struct = np.ones((2 * infl_cells + 1, 2 * infl_cells + 1), dtype=bool)
        inflated = binary_dilation(obstacle, structure=struct)

        # Treat unknown cells (-1) as obstacles too
        unknown = (grid_data < 0)
        inflated = inflated | unknown

        self.get_logger().info(
            f"Planning: start={start} → goal={goal} | "
            f"map {w}×{h} @ {res:.3f} m/cell | "
            f"inflation {infl_cells} cells"
        )

        # Validate start/goal
        for name, (r, c) in [("start", start), ("goal", goal)]:
            if not (0 <= r < h and 0 <= c < w):
                self.get_logger().error(f"{name} {(r,c)} is outside map bounds — aborting plan")
                return
            if inflated[r, c]:
                self.get_logger().warn(
                    f"{name} cell is inside inflated obstacle — nudging to nearest free cell"
                )

        path_cells = self._astar(inflated, start, goal, h, w)

        if path_cells is None:
            self.get_logger().error("A* found no path — check goal coordinates")
            return

        self.get_logger().info(f"A* found {len(path_cells)} cells — pruning...")

        # Line-of-sight pruning: skip waypoints if robot can travel straight between them
        pruned = self._prune(path_cells, inflated, w, h)
        self.get_logger().info(f"Pruned to {len(pruned)} waypoints")

        # Publish as nav_msgs/Path
        path_msg = Path()
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for r, c in pruned:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x, ps.pose.position.y = grid_to_world(r, c)
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        self._path_pub.publish(path_msg)
        self.get_logger().info(
            f"Published planned path with {len(path_msg.poses)} waypoints"
        )

    # ------------------------------------------------------------------
    # A* search
    # ------------------------------------------------------------------

    def _astar(self, obstacle: np.ndarray, start: tuple, goal: tuple,
               h: int, w: int) -> list | None:
        sr, sc = start
        gr, gc = goal

        def heuristic(r, c):
            return math.hypot(r - gr, c - gc)

        # (f, g, r, c, parent)
        open_heap: list = []
        heapq.heappush(open_heap, (heuristic(sr, sc), 0.0, sr, sc, None))

        came_from: dict = {}
        g_score: dict = {(sr, sc): 0.0}
        closed: set = set()

        DIRS = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
        COSTS = [1.0, 1.0, 1.0, 1.0, math.sqrt(2)] * 2  # cardinal/diagonal

        while open_heap:
            f, g, r, c, parent = heapq.heappop(open_heap)

            if (r, c) in closed:
                continue
            closed.add((r, c))
            came_from[(r, c)] = parent

            if (r, c) == (gr, gc):
                # Reconstruct path
                path = []
                node = (gr, gc)
                while node is not None:
                    path.append(node)
                    node = came_from[node]
                path.reverse()
                return path

            for i, (dr, dc) in enumerate(DIRS):
                nr, nc = r + dr, c + dc
                cost = 1.0 if i < 4 else math.sqrt(2)
                if not (0 <= nr < h and 0 <= nc < w):
                    continue
                if obstacle[nr, nc]:
                    continue
                if (nr, nc) in closed:
                    continue
                ng = g + cost
                if ng < g_score.get((nr, nc), float("inf")):
                    g_score[(nr, nc)] = ng
                    heapq.heappush(open_heap,
                                   (ng + heuristic(nr, nc), ng, nr, nc, (r, c)))

        return None  # no path

    # ------------------------------------------------------------------
    # Line-of-sight pruning
    # ------------------------------------------------------------------

    def _prune(self, path: list, obstacle: np.ndarray, w: int, h: int) -> list:
        """Remove intermediate waypoints where a straight line is obstacle-free."""
        if len(path) <= 2:
            return path
        pruned = [path[0]]
        i = 0
        while i < len(path) - 1:
            # Find the furthest point reachable in a straight line from path[i]
            j = len(path) - 1
            while j > i + 1:
                if self._los_clear(path[i], path[j], obstacle, w, h):
                    break
                j -= 1
            pruned.append(path[j])
            i = j
        return pruned

    def _los_clear(self, a: tuple, b: tuple, obstacle: np.ndarray,
                   w: int, h: int) -> bool:
        """Bresenham line-of-sight check between two grid cells."""
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
                r += sr
            if e2 < dr:
                err += dr
                c += sc


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
