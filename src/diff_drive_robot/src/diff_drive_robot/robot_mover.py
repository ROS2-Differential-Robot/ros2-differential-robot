#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher_ = self.create_publisher(Twist, '/diff_drive/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.move_robot)

    def move_robot(self):
        msg = Twist()
        msg.linear.x = 5.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = -0.1
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    robot_mover = RobotMover()
    rclpy.spin(robot_mover)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
