#!/usr/bin/env python3

from __future__ import annotations

import argparse
from collections import deque
import math
from typing import Iterable, List, Optional, Sequence, Tuple


try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None


def _try_import_matplotlib():
    try:
        import matplotlib.pyplot as plt
    except ImportError:  # pragma: no cover
        return None
    return plt


Coord = Tuple[int, int]  # (row, col)


class GridGraph:

    def __init__(self, grid: Sequence[int], width: int, height: int):
        self.width = width
        self.height = height
        self.grid = grid

    def in_bounds(self, r: int, c: int) -> bool:
        return 0 <= r < self.height and 0 <= c < self.width

    def is_free(self, r: int, c: int) -> bool:
        v = self.grid[r * self.width + c]
        return v == 0

     def neighbors(self, r, c):

        directions = [
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ]
        for dr, dc in directions:
            rr, cc = r + dr, c + dc
            if not self.in_bounds(rr, cc):
                continue
            if not self.is_free(rr, cc):
                continue

            if dr != 0 and dc != 0:
                if not (self.in_bounds(r, cc) and self.is_free(r, cc)):
                    continue
                if not (self.in_bounds(rr, c) and self.is_free(rr, c)):
                    continue

            yield (rr, cc)

    def shortest_path_bfs(self, start: Coord, goal: Coord) -> Optional[List[Coord]]:
        sr, sc = start
        gr, gc = goal
        if not (self.in_bounds(sr, sc) and self.in_bounds(gr, gc)):
            return None
        if not (self.is_free(sr, sc) and self.is_free(gr, gc)):
            return None

        q: deque[Coord] = deque()
        q.append(start)
        came_from = {start: None}

        while q:
            curr = q.popleft()
            if curr == goal:
                break
            for nb in self.neighbors(*curr):
                if nb not in came_from:
                    came_from[nb] = curr
                    q.append(nb)

        if goal not in came_from:
            return None
        path: List[Coord] = []
        cur: Optional[Coord] = goal
        while cur is not None:
            path.append(cur)
            cur = came_from[cur]
        path.reverse()
        return path


def inflate_map(data, width, height, radius):
        inflated = data[:]
        for r in range(height):
            for c in range(width):
                if data[r * width + c] >= 50:
                    for dr in range(-radius, radius + 1):
                        for dc in range(-radius, radius + 1):
                            rr = r + dr
                            cc = c + dc
                            if 0 <= rr < height and 0 <= cc < width:
                                idx = rr * width + cc
                                inflated[idx] = max(inflated[idx], 100)
        return inflated


def _grid_to_img(grid: Sequence[int], width: int, height: int):
    if np is not None:
        arr = np.asarray(list(grid), dtype=np.int16).reshape((height, width))
        img = np.ones((height, width), dtype=np.float32)
        img[arr == -1] = 0.5
        img[(arr != 0) & (arr != -1)] = 0.0
        return img

    img = []
    for r in range(height):
        row = []
        base = r * width
        for c in range(width):
            v = grid[base + c]
            if v == 0:
                row.append(1.0)
            elif v == -1:
                row.append(0.5)
            else:
                row.append(0.0)
        img.append(row)
    return img


def _cells_to_world(
    cells: Sequence[Coord],
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> Tuple[List[float], List[float]]:
    xs: List[float] = []
    ys: List[float] = []
    for r, c in cells:
        xs.append(origin_x + (c + 0.5) * resolution)
        ys.append(origin_y + (r + 0.5) * resolution)
    return xs, ys


def _build_demo_grid(width: int, height: int) -> List[int]:
    grid = [0] * (width * height)

    def set_wall(r: int, c: int):
        grid[r * width + c] = 100

    for r in range(height):
        set_wall(r, 0)
        set_wall(r, width - 1)
    for c in range(width):
        set_wall(0, c)
        set_wall(height - 1, c)

    for r in range(2, height - 2):
        set_wall(r, width // 3)
    for c in range(2, width - 2):
        if c == width // 2:
            continue
        set_wall(height // 2, c)

    return grid


def run_demo(args) -> int:
    plt = _try_import_matplotlib()

    width = int(args.demo_width)
    height = int(args.demo_height)
    grid = _build_demo_grid(width, height)
    graph = GridGraph(grid, width, height)
    start = (1, 1)
    goal = (height - 2, width - 2)
    path = graph.shortest_path_bfs(start, goal)
    if path is None:
        print('demo: no path')
        return 0

    img = _grid_to_img(grid, width, height)
    fig, ax = plt.subplots()
    ax.imshow(img, origin='lower', cmap='gray', interpolation='nearest')
    xs, ys = _cells_to_world(path, origin_x=0.0, origin_y=0.0, resolution=1.0)
    ax.plot(xs, ys, 'r-', linewidth=2)
    ax.set_title(f'GridGraph BFS demo (len={len(path)})')
    ax.set_aspect('equal')
    plt.show()
    return 0


class GridGraphVisualizerNode:

    def __init__(self, args):
        import rclpy  # lazy import
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import OccupancyGrid, Odometry, Path
        from rclpy.node import Node

        self._rclpy = rclpy

        class _Node(Node):
            pass

        self.node = _Node('grid_graph_visualizer')

        self.map_topic = args.map_topic
        self.path_topic = args.path_topic
        self.odom_topic = args.odom_topic
        self.goal_topic = args.goal_topic
        self.compute_bfs = bool(args.compute_bfs)
        self.safe_margin = max(0.0, float(args.safe_margin))
        self.recompute_min_period = max(0.0, float(args.recompute_min_period))

        self._map_width: Optional[int] = None
        self._map_height: Optional[int] = None
        self._map_res: Optional[float] = None
        self._origin_x: Optional[float] = None
        self._origin_y: Optional[float] = None
        self._grid: Optional[List[int]] = None

        self._path_xy: Optional[List[Tuple[float, float]]] = None
        self._robot_xy: Optional[Tuple[float, float]] = None
        self._goal_xy: Optional[Tuple[float, float]] = None

        self._last_start_goal: Optional[Tuple[Coord, Coord]] = None
        self._last_compute_time = 0.0
        self._last_bfs_path: Optional[List[Coord]] = None
        self._last_bfs_valid = False

        self.node.create_subscription(OccupancyGrid, self.map_topic, self._on_map, 10)
        self.node.create_subscription(Path, self.path_topic, self._on_path, 10)
        self.node.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.node.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 10)

        plt = _try_import_matplotlib()


        self._plt = plt
        self._fig, self._ax = plt.subplots()
        self._im = None
        (self._path_line,) = self._ax.plot([], [], 'r-', linewidth=2, label='planned_path')
        (self._bfs_line,) = self._ax.plot([], [], 'c--', linewidth=1, label='bfs(local)')
        (self._robot_pt,) = self._ax.plot([], [], 'yo', markersize=4, label='robot')
        (self._goal_pt,) = self._ax.plot([], [], 'bo', markersize=4, label='goal')
        self._ax.set_title('GridGraph Visualizer')
        self._ax.set_xlabel('x')
        self._ax.set_ylabel('y')
        self._ax.set_aspect('equal')
        self._ax.legend(loc='upper right')

        plt.ion()
        plt.show(block=False)

    def destroy(self):
        self.node.destroy_node()

    def _on_map(self, msg):
        self._map_width = int(msg.info.width)
        self._map_height = int(msg.info.height)
        self._map_res = float(msg.info.resolution)
        self._origin_x = float(msg.info.origin.position.x)
        self._origin_y = float(msg.info.origin.position.y)
        self._grid = list(msg.data)

        if self._im is None:
            img = _grid_to_img(self._grid, self._map_width, self._map_height)
            extent = [
                self._origin_x,
                self._origin_x + self._map_width * self._map_res,
                self._origin_y,
                self._origin_y + self._map_height * self._map_res,
            ]
            self._im = self._ax.imshow(
                img,
                origin='lower',
                cmap='gray',
                interpolation='nearest',
                extent=extent,
            )
            self._ax.set_xlim(extent[0], extent[1])
            self._ax.set_ylim(extent[2], extent[3])
        else:
            img = _grid_to_img(self._grid, self._map_width, self._map_height)
            self._im.set_data(img)

        self._fig.canvas.draw_idle()

    def _on_path(self, msg):
        if not msg.poses:
            self._path_xy = None
            return
        self._path_xy = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _on_odom(self, msg):
        self._robot_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    def _on_goal(self, msg):
        self._goal_xy = (float(msg.pose.position.x), float(msg.pose.position.y))

    def _compute_start_goal_cells(self) -> Optional[Tuple[Coord, Coord]]:
        if (
            self._grid is None
            or self._map_width is None
            or self._map_height is None
            or self._map_res is None
            or self._origin_x is None
            or self._origin_y is None
            or self._robot_xy is None
            or self._goal_xy is None
        ):
            return None

        res = self._map_res
        if res <= 0.0:
            return None

        rx, ry = self._robot_xy
        gx, gy = self._goal_xy
        start_col = int((rx - self._origin_x) / res)
        start_row = int((ry - self._origin_y) / res)
        goal_col = int((gx - self._origin_x) / res)
        goal_row = int((gy - self._origin_y) / res)
        return (start_row, start_col), (goal_row, goal_col)

    def _maybe_run_bfs(self):
        if not self.compute_bfs:
            return

        sg = self._compute_start_goal_cells()
        if sg is None:
            return

        start, goal = sg
        now = float(self.node.get_clock().now().nanoseconds) * 1e-9
        if (self._last_start_goal == sg) and (
            now - self._last_compute_time < self.recompute_min_period
        ):
            return

        self._last_start_goal = sg
        self._last_compute_time = now

        assert self._grid is not None
        assert self._map_width is not None
        assert self._map_height is not None
        assert self._map_res is not None

        radius_cells = int(math.ceil(self.safe_margin / self._map_res))
        grid = inflate_map(self._grid, self._map_width, self._map_height, radius_cells)
        graph = GridGraph(grid, self._map_width, self._map_height)
        self._last_bfs_path = graph.shortest_path_bfs(start, goal)
        self._last_bfs_valid = True

    def render_once(self):
        self._maybe_run_bfs()

        if self._path_xy:
            xs = [p[0] for p in self._path_xy]
            ys = [p[1] for p in self._path_xy]
            self._path_line.set_data(xs, ys)
        else:
            self._path_line.set_data([], [])

        if self._last_bfs_valid and self._last_bfs_path and self._map_res is not None:
            assert self._origin_x is not None
            assert self._origin_y is not None
            xs, ys = _cells_to_world(
                self._last_bfs_path,
                self._origin_x,
                self._origin_y,
                self._map_res,
            )
            self._bfs_line.set_data(xs, ys)
        else:
            self._bfs_line.set_data([], [])

        if self._robot_xy is not None:
            self._robot_pt.set_data([self._robot_xy[0]], [self._robot_xy[1]])
        if self._goal_xy is not None:
            self._goal_pt.set_data([self._goal_xy[0]], [self._goal_xy[1]])

        self._plt.pause(0.001)


def _parse_args(argv: Optional[Sequence[str]] = None):
    parser = argparse.ArgumentParser(description='GridGraph visualization tool')

    parser.add_argument('--demo', action='store_true', help='Offline demo (does not require ROS2)')
    parser.add_argument('--demo-width', type=int, default=40, help='Demo map width')
    parser.add_argument('--demo-height', type=int, default=25, help='Demo map height')

    parser.add_argument('--map-topic', default='maze_map', help='OccupancyGrid topic name')
    parser.add_argument('--path-topic', default='planned_path', help='Path topic name')
    parser.add_argument('--odom-topic', default='/odom', help='Odometry topic name')
    parser.add_argument('--goal-topic', default='/goal_pose', help='Goal Pose topic name')

    parser.add_argument(
        '--compute-bfs',
        action='store_true',
        help='Do not rely on planned_path; compute a path yourself using GridGraph (BFS) and draw it',
    )
    parser.add_argument('--safe-margin', type=float, default=0.25, help='Obstacle inflation radius (meters)')
    parser.add_argument(
        '--recompute-min-period',
        type=float,
        default=0.2,
        help='Minimum recomputation period (seconds) to avoid running BFS too frequently',
    )

    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    if args.demo:
        return run_demo(args)

    import rclpy

    rclpy.init(args=None)
    node = GridGraphVisualizerNode(args)
    try:
        while rclpy.ok():
            rclpy.spin_once(node.node, timeout_sec=0.05)
            node.render_once()
            if node._plt is not None and not node._plt.fignum_exists(node._fig.number):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
