import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import yaml
import os

class GoalSender(Node):

    def __init__(self):
        super().__init__('goal_sender')
        self.subscription = self.create_subscription(
            String,
            'table_num',  # Topic to listen for the target ID
            self.target_callback,
            10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        with open(os.path.join(os.getcwd(), 'src/ros2-differential-robot/src/restaurant_nav/restaurant_nav/positions.yaml'), 'r') as file:
            self.positions = yaml.safe_load(file)['positions']
        self.get_logger().info('GoalSender node started.')

    def target_callback(self, msg):
        target_id = int(msg.data)
        if target_id in self.positions:
            x, y, z, w = self.positions[target_id]
            self.send_goal(x, y, z, w)
        else:
            self.get_logger().warn(f"Invalid ID received: {target_id}")

    def send_goal(self, x, y, z, w):
        goal = PoseStamped()
        goal.header.frame_id = "map"  # Ensure this matches your setup
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = z 
        goal.pose.orientation.w = w 
        self.goal_publisher.publish(goal)
        self.get_logger().info(f"Sent goal: x={x:.2f}, y={y:.2f}, z={z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
