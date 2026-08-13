#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.qos import QoSProfile


class TwistToAckermann(Node):
    def __init__(self):
        super().__init__('twist_to_ackermann')
        self.ack_pub = self.create_publisher(AckermannDriveStamped, '/medibot_ackermann_cmd', QoSProfile(depth=10))
        self.cmd_sub = self.create_subscription(Twist, 'medibot_cmd_vel', self.on_cmd_vel, QoSProfile(depth=10))
        self.wheelbase = 0.143
        self.frame_id = 'medibot_odom_combined'
        self.angle_mode = False

    def compute_steering(self, linear_vel, angular_vel):
        if angular_vel == 0 or linear_vel == 0:
            return 0.0
        turn_radius = linear_vel / angular_vel
        return math.atan(self.wheelbase / turn_radius)

    def on_cmd_vel(self, msg):
        forward_speed = msg.linear.x
        if self.angle_mode:
            steer_angle = msg.angular.z
        else:
            steer_angle = self.compute_steering(forward_speed, msg.angular.z)

        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.drive.steering_angle = steer_angle
        out.drive.speed = forward_speed
        self.ack_pub.publish(out)


def main():
    rclpy.init()
    node = TwistToAckermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
