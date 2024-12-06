import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistStamper(Node):
    def __init__(self):
        super().__init__("twist_stamper")
        self.__publisher = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self.stamp, 10)

    def stamp(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.twist = msg
        self.__publisher.publish(stamped)


def main():
    rclpy.init()
    node = TwistStamper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
