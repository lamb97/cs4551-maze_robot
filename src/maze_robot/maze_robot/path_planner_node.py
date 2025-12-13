import math
from collections import deque

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class GridGraph:
    def __init__(self, grid, width, height):
        self.width = width
        self.height = height
        self.grid = grid

    def in_bounds(self, r, c):
        return 0 <= r < self.height and 0 <= c < self.width

    def is_free(self, r, c):
        v = self.grid[r * self.width + c]
        return v == 0 

    def neighbors(self, r, c):
        # 4-connected
        for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            rr, cc = r + dr, c + dc
            if self.in_bounds(rr, cc) and self.is_free(rr, cc):
                yield (rr, cc)

    def shortest_path_bfs(self, start, goal):
        sr, sc = start
        gr, gc = goal
        if not (self.in_bounds(sr, sc) and self.in_bounds(gr, gc)):
            return None
        if not (self.is_free(sr, sc) and self.is_free(gr, gc)):
            return None

        q = deque()
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
        path = []
        cur = goal
        while cur is not None:
            path.append(cur)
            cur = came_from[cur]
        path.reverse()
        return path


class MazePlanner(Node):
    def __init__(self):
        super().__init__('maze_planner')
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'maze_map',          # TODO: 
            self.map_callback,
            10
        )
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.has_planned = False

    def map_callback(self, msg: OccupancyGrid):
        if self.has_planned:
            return

        self.get_logger().info('Received map, start planning...')

        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution
        origin = msg.info.origin  # geometry_msgs/Pose

        graph = GridGraph(msg.data, width, height)
        start = (1, 1)                 # row, col
        goal = (height - 2, width - 2) # row, col

        path_cells = graph.shortest_path_bfs(start, goal)
        if path_cells is None:
            self.get_logger().warn('No path found from start to goal.')
            return

        self.get_logger().info(f'Path length (cells): {len(path_cells)}')
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = msg.header.frame_id or 'map'

        for (r, c) in path_cells:
            px = origin.position.x + (c + 0.5) * res
            py = origin.position.y + (r + 0.5) * res

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0 
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.has_planned = True
        self.get_logger().info('Published planned_path.')


def main(args=None):
    rclpy.init(args=args)
    node = MazePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


