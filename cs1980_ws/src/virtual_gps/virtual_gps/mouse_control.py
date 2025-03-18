import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32

import time

class RaspimouseMover(Node):

    def __init__(self):
        super().__init__('raspimouse_mover')

        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value

        self.robot_pose = TransformStamped()

        self.robot_orientation = 0.0

        self.vel_publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # get position from the gps node to guide movement 
        self.pose_subscriber = self.create_subscription(
            TransformStamped, 
            f'/{self.robot_name}/gps',
            self.pose_callback,
            10
        )
        self.pose_subscriber 

        self.orientation_subscriber = self.create_subscription(
            Float32,
            f'/{self.robot_name}/orientation_degrees',
            self.orientation_callback,
            10
        )

    
    def pose_callback(self, msg):
        self.robot_pose = msg

    def orientation_callback(self, msg):
        self.robot_orientation = msg.data


    def move(self, linear_speed=0.1, angular_speed=0.0):
        """Publish velocity command for a given duration."""
        twist = TwistStamped()
        twist.twist.linear.x = linear_speed
        twist.twist.angular.z = angular_speed
        self.vel_publisher.publish(twist)


    def stop(self):
        """Stop the robot."""
        twist = TwistStamped()
        self.vel_publisher.publish(twist)


    def execute_movement(self):
        """
        while True:
            self.move(angular_speed=0.5) 
            rclpy.spin_once(self)
            time.sleep(0.2) 
        """
        #Moves the robot forward 1m, turns left, and moves forward again.
        self.get_logger().info("Moving forward 1 meter...")
        while self.robot_pose.transform.translation.x < 1.0:
            self.move(linear_speed=0.2) 
            rclpy.spin_once(self)
        self.stop()

        self.get_logger().info("Turning left 90 degrees...")
        while self.robot_orientation < 85.0:
            self.move(angular_speed=0.25) 
            rclpy.spin_once(self)
        self.stop()

        self.get_logger().info("Moving forward 1 meter in new direction...")
        while self.robot_pose.transform.translation.y < 1.0:
            self.move(linear_speed=0.2)
            rclpy.spin_once(self)
        self.stop()

        self.get_logger().info("Turning left 90 degrees...")
        while self.robot_orientation < 175.0:
            self.move(angular_speed=0.25) 
            rclpy.spin_once(self)
        self.stop()

        self.get_logger().info("Moving forward 1 meter in new direction...")
        while self.robot_pose.transform.translation.x > 0.0:
            self.move(linear_speed=0.2)
            rclpy.spin_once(self)
        self.stop()

        self.get_logger().info("Turning left 90 degrees...")
        while self.robot_orientation > 175.0 or self.robot_orientation < -95.0:
            self.move(angular_speed=0.25) 
            rclpy.spin_once(self)
        self.stop()

        self.get_logger().info("Moving forward 1 meter in new direction...")
        while self.robot_pose.transform.translation.y > 0.0:
            self.move(linear_speed=0.2)
            rclpy.spin_once(self)
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
