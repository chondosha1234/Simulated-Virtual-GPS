import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped

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
        self.error_publisher = self.create_publisher(Float32, f'/{self.robot_name}/error_measure', 1)

        self.raspimouse_sub = self.create_subscription(TransformStamped, f'/model/{self.robot_name}/pose', self.raspimouse_pose_callback, 1)
        self.raspimouse_sub

        # subscriber to topic '/gps' which contains estimated position of robot
        self.gps_subscription = self.create_subscription(TransformStamped, f'/{self.robot_name}/gps', self.gps_callback, 1)
        self.gps_subscription

        # timer that runs callback function every 200ms
        self.timer = self.create_timer(.25, self.timer_callback)

        self.counter = 0
        self.bad_counter = 0


    def raspimouse_pose_callback(self,msg):
        self.robot_actual_pose = msg

    def gps_callback(self, msg):
        # setting pose variable equal to msg (should be of type TransformStamped)
        self.robot_gps_pose = msg
    

    def timer_callback(self):

        # calculate error between actual ground robot location and gps location
        error = self.calculate_error(self.robot_gps_pose, self.robot_actual_pose)
        self.counter += 1

        # here for testing purposes 
        self.get_logger().info(f'error distance calc: {error}')
        
        # publish error calculation to topic
        if not math.isnan(error) and abs(error) < 50:
            msg = Float32()
            msg.data = error
            self.error_publisher.publish(msg)
        else:
            self.bad_counter += 1
        
        self.get_logger().info(f'{self.bad_counter} of {self.counter} were off by more than 50 meters')
    

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
