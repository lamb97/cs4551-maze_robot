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
        self.goal_tol = 0.10      
        self.max_lin = 0.18
        self.max_ang = 1.0

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

        tx, ty = self.path_points[self.idx]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        if dist < self.dist_tol and self.idx < len(self.path_points) - 1:
            self.idx += 1
            tx, ty = self.path_points[self.idx]
            dx = tx - self.x
            dy = ty - self.y
            dist = math.hypot(dx, dy)

        if self.idx == len(self.path_points) - 1 and dist < self.goal_tol:
            self.get_logger().info("Reached goal, stopping.")
            self.path_points = []
            self.publish_stop()
            return

        target_yaw = math.atan2(dy, dx)
        yaw_err = normalize_angle(target_yaw - self.yaw)

        cmd = Twist()

        angle_threshold = 0.3  # rad
        if abs(yaw_err) > angle_threshold:
            cmd.linear.x = 0.0
            cmd.angular.z = max(-self.max_ang, min(self.max_ang, 1.5 * yaw_err))
        else:
            cmd.linear.x = min(self.max_lin, 0.6 * dist)
            cmd.angular.z = max(-self.max_ang, min(self.max_ang, 1.0 * yaw_err))

        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SimplePathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
