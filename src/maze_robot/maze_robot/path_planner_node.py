import math
from collections import deque

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path, Odometry
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
        # for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        for dr, dc in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
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
        self.safe_margin = 0.25  # meters to keep from walls

        odom_topic = self.declare_parameter(
            'odom_topic',
            '/odom'
        ).value
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.goalpose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goalpose_callback,
            10
        )


        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.has_map = False  
        self.has_planned = False
        self.current_map = None
        self.map_info = None
        self.robot_pose = None
        self.goal_pose = None

    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().info('Received map')
        self.has_map = True
        self.map_info = msg.info
        raw_map = list(msg.data)
        radius_cells = max(1, int(math.ceil(self.safe_margin / self.map_info.resolution)))
        inflated = self.inflate_map(
            raw_map,
            self.map_info.width,
            self.map_info.height,
            radius_cells,
        )
        self.current_map = inflated
        
        if self.robot_pose and self.goal_pose  :
            self.plan_path()

        

    def goalpose_callback(self, msg: PoseStamped):
        self.goal_pose = msg.pose
        self.get_logger().info(f'Goal Pose: ({self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f})')
        if self.has_map and self.robot_pose:
            self.plan_path()

    def odom_callback(self, msg: Odometry):
        self.robot_pose = msg.pose.pose
        if self.has_map and self.goal_pose:
            self.get_logger().info(
                f"Robot pose from odom: ({self.robot_pose.position.x:.2f}, {self.robot_pose.position.y:.2f})"
            )
            self.plan_path()
            
    def plan_path(self): 
        if self.has_planned:
            return

        self.get_logger().info('Received map, start planning...')

        width = self.map_info.width
        height = self.map_info.height
        res = self.map_info.resolution
        origin = self.map_info.origin  # geometry_msgs/Pose

        graph = GridGraph(self.current_map, width, height)
        if self.robot_pose and self.goal_pose:
            start_col = int((self.robot_pose.position.x - origin.position.x) / res)
            start_row = int((self.robot_pose.position.y - origin.position.y) / res)

            goal_col = int((self.goal_pose.position.x - origin.position.x) / res)
            goal_row = int((self.goal_pose.position.y - origin.position.y) / res)
            start = (start_row, start_col)
            goal = (goal_row, goal_col)
        else:
            start = (1, 1)
            goal = (height - 2, width - 2)
        path_cells = graph.shortest_path_bfs(start, goal)
        if path_cells is None:
            self.get_logger().warn('No path found from start to goal.')
            return
        self.get_logger().info(f'Path length (cells): {len(path_cells)}')
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map' 

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

    @staticmethod
    def inflate_map(data, width, height, radius):
        inflated = data[:]
        for r in range(height):
            for c in range(width):
                if data[r * width + c] != 0:
                    for dr in range(-radius, radius + 1):
                        for dc in range(-radius, radius + 1):
                            rr = r + dr
                            cc = c + dc
                            if 0 <= rr < height and 0 <= cc < width:
                                inflated[rr * width + cc] >= 100 
        return inflated

def main(args=None):
    rclpy.init(args=args)
    node = MazePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
