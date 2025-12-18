#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class SimplePathFollower(Node):
    def __init__(self):
        super().__init__('simple_path_follower')

        self.path_sub = self.create_subscription(
            Path, 'planned_path', self.path_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.path_points = []
        self.idx = 0
        self.has_odom = False

        self.dist_tol = 0.08
        self.goal_tol = 0.12
        self.max_lin = 0.12
        self.max_ang = 0.8
        self.lookahead = 0.35

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("SimplePathFollower started.")

    def path_callback(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn("Empty path received.")
            self.path_points = []
            return
        self.path_points = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]
        self.idx = 0
        self.get_logger().info(f"New path with {len(self.path_points)} points.")

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.has_odom = True

    def control_loop(self):
        if not self.has_odom or not self.path_points:
            self.publish_stop()
            return

        current_target = self._advance_index()
        dist_to_target = math.hypot(
            current_target[0] - self.x,
            current_target[1] - self.y
        )

        if self.idx == len(self.path_points) - 1 and dist_to_target < self.goal_tol:
            self.get_logger().info("Reached goal, stopping.")
            self.path_points = []
            self.publish_stop()
            return

        target_point = self._lookahead_point()
        lx, ly, lookahead_dist = self._transform_to_base(target_point)
        if lookahead_dist < 1e-3:
            self.publish_stop()
            return

        if lx <= 0.0 and self.idx < len(self.path_points) - 1:
            # advance further until point is in front
            self.idx += 1
            return

        curvature = 2.0 * ly / (lookahead_dist ** 2)
        cmd = Twist()
        cmd.angular.z = max(-self.max_ang, min(self.max_ang, curvature * self.max_lin))

        if abs(ly) > 0.35 * lookahead_dist or abs(cmd.angular.z) > 0.45:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = min(self.max_lin, lookahead_dist) * 0.8

        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _advance_index(self):
        while self.idx < len(self.path_points) - 1:
            px, py = self.path_points[self.idx]
            if math.hypot(px - self.x, py - self.y) < self.dist_tol:
                self.idx += 1
                continue
            if not self._is_point_in_front((px, py)):
                self.idx += 1
            else:
                break
        return self.path_points[self.idx]

    def _lookahead_point(self):
        if not self.path_points:
            return (self.x, self.y)
        target_idx = self.idx
        chosen = self.path_points[self.idx]
        while target_idx < len(self.path_points):
            px, py = self.path_points[target_idx]
            dist = math.hypot(px - self.x, py - self.y)
            if dist >= self.lookahead and self._is_point_in_front((px, py)):
                chosen = (px, py)
                break
            target_idx += 1
        else:
            chosen = self.path_points[-1]
        return chosen

    def _transform_to_base(self, point):
        dx = point[0] - self.x
        dy = point[1] - self.y
        sin_yaw = math.sin(self.yaw)
        cos_yaw = math.cos(self.yaw)
        lx = cos_yaw * dx + sin_yaw * dy
        ly = -sin_yaw * dx + cos_yaw * dy
        return lx, ly, math.hypot(lx, ly)

    def _is_point_in_front(self, point):
        lx, _, _ = self._transform_to_base(point)
        return lx > 0.05


def main(args=None):
    rclpy.init(args=args)
    node = SimplePathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
