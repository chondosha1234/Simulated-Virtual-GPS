import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32

import tf_transformations
import math

class OrientationNode(Node):

    def __init__(self):
        super().__init__('orientation_node')

        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value
        self.get_logger().info(f'robot name in orientation detection: {self.robot_name}')

        self.orientation_publisher = self.create_publisher(Float32, f'/{self.robot_name}/orientation_degrees', 10)

        self.robot_pose = TransformStamped() 

        self.robot_pose_sub = self.create_subscription(TransformStamped, f'/model/{self.robot_name}/pose', self.pose_callback, 10)
        self.robot_pose_sub 

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def pose_callback(self, msg):
        self.robot_pose = msg


    def timer_callback(self):

        orient_deg = Float32()

        qx = self.robot_pose.transform.rotation.x
        qy = self.robot_pose.transform.rotation.y
        qz = self.robot_pose.transform.rotation.z
        qw = self.robot_pose.transform.rotation.w

        # Convert quaternion to Euler angles (radians)
        roll, pitch, yaw = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])

        # Convert radians to degrees
        yaw = math.degrees(yaw)
        orient_deg.data = float(yaw)
        #self.get_logger().info(f'degrees: {yaw}')

        self.orientation_publisher.publish(orient_deg)
    


def main(args=None):
    rclpy.init(args=args)
    node = OrientationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()