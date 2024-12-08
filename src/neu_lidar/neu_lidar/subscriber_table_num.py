import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class TableNumSubscriber(Node):

    def __init__(self):
        super().__init__('table_num_subscriber')
        self.subscription = self.create_subscription(
            String,
            'table_num',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main():
    rclpy.init()
    table_num_subscriber = TableNumSubscriber()
    rclpy.spin(table_num_subscriber)
    table_num_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()