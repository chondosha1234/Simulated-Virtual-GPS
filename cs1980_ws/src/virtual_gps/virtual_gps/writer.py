import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from std_msgs.msg import Float32
import math
import numpy as np

import json
import signal
import os
from ament_index_python.packages import get_package_share_directory

# variables for writing to json files
actual_data = []
calc_data = []
kalman_data = []
error_data = []
kalman_error_data = []

package_path = get_package_share_directory('virtual_gps')
calc_filepath = os.path.join(os.getcwd(), 'src', 'virtual_gps', 'virtual_gps', 'measurements', 'calc_data.json')
actual_filepath = os.path.join(os.getcwd(), 'src', 'virtual_gps', 'virtual_gps', 'measurements', 'actual_data.json')
error_filepath = os.path.join(os.getcwd(), 'src', 'virtual_gps', 'virtual_gps', 'measurements', 'error_data.json')

kalman_filepath = os.path.join(os.getcwd(), 'src', 'virtual_gps', 'virtual_gps', 'measurements', 'kalman_data.json')
kalman_error_filepath = os.path.join(os.getcwd(), 'src', 'virtual_gps', 'virtual_gps', 'measurements', 'kalman_error_data.json')


class WriterNode(Node):
    
    def __init__(self):
        super().__init__('json_writer')

        # ground robot name
        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value

        # keeps track of ground robot's actual position
        self.robot_actual_pose = TransformStamped()

        # keeps track of ground robot's gps position
        self.robot_gps_pose = TransformStamped()

        self.filter_gps_pose = TransformStamped()

        # keeps track of error measurement
        self.error = Float32()

        self.kalman_error = Float32()

        # subscriber to topic which contains actual position of robot
        self.raspimouse_sub = self.create_subscription(TransformStamped, f'/model/{self.robot_name}/pose', self.raspimouse_pose_callback, 10)
        self.raspimouse_sub

        # subscriber to topic '/gps' which contains estimated position of robot
        self.gps_subscription = self.create_subscription(TransformStamped, f'/{self.robot_name}/gps', self.gps_callback, 10)
        self.gps_subscription

        # subscriber to topic '/error_measure'
        self.error_subscription = self.create_subscription(Float32, f'/{self.robot_name}/error_measure', self.error_callback, 10)
        self.error_subscription

        self.kalman_filter_sub = self.create_subscription(TransformStamped, f'/{self.robot_name}/filtered_gps', self.kalman_callback, 10)
        self.kalman_filter_sub

        self.kalman_error_sub = self.create_subscription(Float32, f'/{self.robot_name}/kalman_error', self.kalman_error_callback, 10)
        self.kalman_error_sub

        # timer that runs callback function every 200ms
        self.timer = self.create_timer(0.2, self.timer_callback)

    def raspimouse_pose_callback(self,msg):
        #if msg.child_frame_id == 'raspimouse':
        self.robot_actual_pose = msg
        

    def gps_callback(self, msg):
        # setting pose variable equal to msg (should be of type TransformStamped)
        self.robot_gps_pose = msg


    def error_callback(self, msg):
        self.error = msg


    def kalman_callback(self, msg):
        self.filter_gps_pose = msg 


    def kalman_error_callback(self, msg):
        self.kalman_error = msg 

    
    def timer_callback(self):
        # getting actual positions
        x1 = self.robot_actual_pose.transform.translation.x
        y1 = self.robot_actual_pose.transform.translation.y
        z1 = self.robot_actual_pose.transform.translation.z

        if not math.isnan(x1) and not math.isnan(y1) and not math.isnan(z1):
            # putting actual position into json format
            a_data = {
                "x": x1,
                "y": y1,
                "z": z1,
            } 

            actual_data.append(a_data)

        # getting calculated positions
        x2 = self.robot_gps_pose.transform.translation.x
        y2 = self.robot_gps_pose.transform.translation.y
        z2 = self.robot_gps_pose.transform.translation.z

        if not math.isnan(x2) and not math.isnan(y2) and not math.isnan(z2):
            # putting calculated position into json format
            c_data = {
                "x": x2,
                "y": y2,
                "z": z2,
            }
            calc_data.append(c_data)

        if not math.isnan(self.error.data):
            # putting error measurement into json format
            e_data = {
                "error": self.error.data
            }
            error_data.append(e_data)

        # get the kalman filtered gps point 
        x3 = self.filter_gps_pose.transform.translation.x
        y3 = self.filter_gps_pose.transform.translation.y
        z3 = self.filter_gps_pose.transform.translation.z
        
        if not math.isnan(x3) and not math.isnan(y3) and not math.isnan(z3):
            # putting calculated position into json format
            k_data = {
                "x": x3,
                "y": y3,
                "z": z3,
            }
            kalman_data.append(k_data)

        if not math.isnan(self.kalman_error.data):
            # putting error measurement into json format
            ke_data = {
                "error": self.kalman_error.data
            }
            kalman_error_data.append(ke_data)
            
        

def write():

    # write to actual_data.json
    with open(actual_filepath, 'w') as json_file:
        json.dump(actual_data, json_file, indent=4)
    
    # write to calc_data.json
    with open(calc_filepath, 'w') as json_file:
        json.dump(calc_data, json_file, indent=4)

    # write to error_data.json
    with open(error_filepath, 'w') as json_file:
        json.dump(error_data, json_file, indent=4)

    # write to error_data.json
    with open(kalman_filepath, 'w') as json_file:
        json.dump(kalman_data, json_file, indent=4)

    # write to error_data.json
    with open(kalman_error_filepath, 'w') as json_file:
        json.dump(kalman_error_data, json_file, indent=4)


def shutdown_handler(signum, frame):
    write()
    rclpy.shutdown()
    exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = WriterNode()

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
