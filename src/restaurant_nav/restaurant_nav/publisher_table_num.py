import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import socket


class TableNumPublisher(Node):

    def __init__(self):
        super().__init__('table_num_publisher')
        self.connect_to_socket()
        self.publisher_ = self.create_publisher(String, 'table_num', 10)
        timer_period = 0.001  # seconds
        # timer_period = 10 # seconds
        self.timer = self.create_timer(timer_period, self.read_from_socket)
    
    def connect_to_socket(self):
        self.LISTEN_PORT = 12345
        self.BUFFER_SIZE = 1024
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('0.0.0.0', self.LISTEN_PORT))
        self.server_socket.listen(1)

    def read_from_socket(self):
        client_socket, _ = self.server_socket.accept()
        table_num = client_socket.recv(self.BUFFER_SIZE)
        if table_num:
            self.publish_table_num(table_num)
        client_socket.close()
    
    def publish_table_num(self, table_num):
        msg = String()
        msg.data = table_num.decode()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main():
    rclpy.init()
    table_num_publisher = TableNumPublisher()
    rclpy.spin(table_num_publisher)
    table_num_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()