import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SetInitialPose


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__("initial_pose_publisher")
        self.__client = self.create_client(SetInitialPose, '/set_initial_pose')
        while not self.__client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting...")

        self.__request = SetInitialPose.Request()

    def send_request(self):
        self.__request.pose.header.stamp = self.get_clock().now().to_msg()
        self.__request.pose.header.frame_id = "map"
        self.get_logger().info("Sending initial pose")
        return self.__client.call_async(self.__request)

def main():
    rclpy.init()
    node = InitialPosePublisher()
    future = node.send_request()
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    node.get_logger().info("response: " + result.__str__())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
