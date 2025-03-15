import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TransformStamped

import tf_transformations
import numpy as np
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

    
    def pose_callback(self, msg):
        self.robot_pose = msg

        q = msg.transform.rotation
        quaternion = [q.x, q.y, q.z, q.w]

        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)

        self.robot_orientation = np.rad2deg(yaw)

        #self.get_logger().info(f'yaw degrees = {self.robot_orientation}')


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

        while True:
            self.move(angular_speed=0.5) 
            rclpy.spin_once(self)
            time.sleep(0.2) 
        """
        Moves the robot forward 1m, turns left, and moves forward again.
        self.get_logger().info("Moving forward 1 meter...")
        while self.robot_pose.transform.translation.x < 1.0:
        #for i in range(50):
            #self.get_logger().info(f'x pose: {self.robot_pose.transform.translation.x}')
            self.move(linear_speed=0.2) 
            rclpy.spin_once(self)
        self.stop()

        self.get_logger().info("Turning left 90 degrees...")
        #while self.robot_orientation < 90.0:
        for i in range(15):
            self.move(angular_speed=0.5) 
            rclpy.spin_once(self)
            time.sleep(0.2)  # send msg every 200ms for 3 seconds 
        self.stop()

        self.get_logger().info("Moving forward 1 meter in new direction...")
        while self.robot_pose.transform.translation.y > -1.0:
        #for i in range(50):
            #self.get_logger().info(f'y pose: {self.robot_pose.transform.translation.y}')
            self.move(linear_speed=-0.1)
            rclpy.spin_once(self)
        self.stop()

        self.get_logger().info("Movement sequence complete.")
        """


def main(args=None):
    rclpy.init(args=args)
    node = RaspimouseMover()
    node.execute_movement()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
