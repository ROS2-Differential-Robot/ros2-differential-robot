import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import os

class PoseRecorder(Node):

    def __init__(self):
        super().__init__('pose_recorder')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',  # Adjust topic name based on your localization method
            self.pose_callback,
            10)
        self.next_id = 1
        self.positions = {}
        self.file_path = os.path.join(os.getcwd(), 'positions.yaml')
        self.get_logger().info('PoseRecorder node started. Listening for robot poses.')

    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        z = msg.pose.orientation.z
        w = msg.pose.orientation.w
        self.positions[self.next_id] = [x,y,z,w]
        self.get_logger().info(f"Recorded position: x={x:.2f}, y={y:.2f}, z={z:.2f} with id={self.next_id}")
        self.next_id += 1
        self.save_poses_to_file()

    def save_poses_to_file(self):
        try:
            with open(self.file_path, 'w') as file:
                yaml.dump({'positions': self.positions}, file)
            self.get_logger().info(f"All positions saved to file: {self.file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save positions to file: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
