import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RaspimouseMover(Node):
    def __init__(self):
        super().__init__('raspimouse_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(1)  # Allow time for ROS 2 setup

    def move(self, linear_speed=0.1, angular_speed=0.0, duration=1.0):
        """Publish velocity command for a given duration."""
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.publisher_.publish(twist)
        time.sleep(duration)

    def stop(self):
        """Stop the robot."""
        twist = Twist()
        self.publisher_.publish(twist)
        time.sleep(1)

    def execute_movement(self):
        """Moves the robot forward 1m, turns left, and moves forward again."""
        self.get_logger().info("Moving forward 1 meter...")
        self.move(linear_speed=0.1, duration=10)  # 0.1 m/s * 10s = 1 meter
        self.stop()

        self.get_logger().info("Turning left 90 degrees...")
        self.move(angular_speed=0.5, duration=3)  # Adjust timing for 90-degree turn
        self.stop()

        self.get_logger().info("Moving forward 1 meter in new direction...")
        self.move(linear_speed=0.1, duration=10)
        self.stop()

        self.get_logger().info("Movement sequence complete.")

def main(args=None):
    rclpy.init(args=args)
    node = RaspimouseMover()
    node.execute_movement()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
