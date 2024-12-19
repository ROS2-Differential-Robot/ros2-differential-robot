import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
import math


class JoystickTwist(Node):
    def __init__(self):
        super().__init__("joy")
        self.__subscriber = self.create_subscription(Joy, '/joy', self.publish_twist, 1)
        self.__publisher = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 1)
        self.__deadzone = 0.1
        self.__stopped = True

    def publish_twist(self, msg: Joy):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()

        changed = False
    
        if not math.isclose(msg.axes[1], 0.0, rel_tol=self.__deadzone):
            twist_stamped.twist.linear.x = msg.axes[1] * 1.0
            changed = True
            self.__stopped = False

        left = -msg.axes[2] / 2 + 0.5
        right = -msg.axes[-3] / 2 + 0.5

        if not math.isclose(left - right, 0.0, rel_tol=self.__deadzone):
            twist_stamped.twist.angular.z = (left - right) * 0.5
            changed = True
            self.__stopped = False

        if not changed and self.__stopped:
            return
        elif not changed and not self.__stopped:
            self.__stopped = True

        self.__publisher.publish(twist_stamped)

def main():
    rclpy.init()
    node = JoystickTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
