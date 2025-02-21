import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from std_msgs.msg import Float32
import math
import numpy as np

class ErrorMeasureNode(Node):
    
    def __init__(self):
        super().__init__('error_measure')

        # ground robot name
        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value

        # keeps track of ground robot's actual position
        self.robot_actual_pose = TransformStamped()

        # keeps track of ground robot's gps position
        self.robot_gps_pose = TransformStamped()
        
        # publisher to topic that ? will subscribe to 
        self.error_publisher = self.create_publisher(Float32, '/error_measure', 10)

        # subscriber to topic '/tf' which contains robot positions
        self.robot_subscription = self.create_subscription(TFMessage, '/tf', self.pose_callback, 10)
        self.robot_subscription

        # subscriber to topic '/gps' which contains estimated position of robot
        self.gps_subscription = self.create_subscription(TransformStamped, '/gps', self.error_callback, 10)
        self.gps_subscription

        # timer that runs callback function every 200ms
        self.timer = self.create_timer(0.2, self.timer_callback)


    def pose_callback(self, msg):
        for transform in msg.transforms:
            name = transform.child_frame_id

            #print(f"transform name: {name}")
        
            # setting pose variable equal to msg (should be of type TransformStamped)
            if name == self.robot_name:
                self.robot_actual_pose = transform
    

    def error_callback(self, msg):
        # setting pose variable equal to msg (should be of type TransformStamped)
        self.robot_gps_pose = msg
    

    def timer_callback(self):
        self.get_logger().info('error measurement timer callback')

        # calculate error between actual ground robot location and gps location
        error = self.calculate_error(self.robot_gps_pose, self.robot_actual_pose)

        # here for testing purposes 
        self.get_logger().info(f'sensor distance calc: {error}')
        
        # publish error calculation to topic
        msg = Float32()
        msg.data = error
        self.error_publisher.publish(msg)
    

    def calculate_error(self, gps_pose, actual_pose):
        # calculation is currently euclidean distance
        # am looking into potentially better error measurements for 3d points
        x1 = gps_pose.transform.translation.x
        y1 = gps_pose.transform.translation.y
        z1 = gps_pose.transform.translation.z

        x2 = actual_pose.transform.translation.x
        y2 = actual_pose.transform.translation.y
        z2 = actual_pose.transform.translation.z

        # calculate euclidean distance using x,y,z values
        x_value = (x1-x2) ** 2
        y_value = (y1-y2) ** 2
        z_value = (z1-z2) ** 2
        euclidean_distance = math.sqrt((x_value + y_value + z_value)) 
        
        return euclidean_distance


def main(args=None):
    rclpy.init(args=args)
    node = ErrorMeasureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
