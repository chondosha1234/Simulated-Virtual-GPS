import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from std_msgs.msg import Float32
import math
import numpy as np

import json


# variables for writing to json files
actual_data = []
calc_data = []
actual_filepath = "actual_data.json"
calc_filepath = "calc_data.json"

class WriterNode(Node):
    
    def __init__(self):
        super().__init__('json_writer')

        # ground robot name
        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value

        # keeps track of ground robot's actual position
        self.robot_actual_pose = TransformStamped()

        # keeps track of ground robot's gps position
        self.robot_gps_pose = TransformStamped()

        # subscriber to topic which contains actual position of robot
        self.raspimouse_sub = self.create_subscription(TransformStamped, f'/model/{self.robot_name}/pose', self.raspimouse_pose_callback, 10)
        self.raspimouse_sub

        # subscriber to topic '/gps' which contains estimated position of robot
        self.gps_subscription = self.create_subscription(TransformStamped, f'/{self.robot_name}/gps', self.error_callback, 10)
        self.gps_subscription

        # timer that runs callback function every 200ms
        self.timer = self.create_timer(0.2, self.timer_callback)

    def raspimouse_pose_callback(self,msg):
        if msg.child_frame_id == 'raspimouse':
            self.robot_pose = msg

    def error_callback(self, msg):
        # setting pose variable equal to msg (should be of type TransformStamped)
        self.robot_gps_pose = msg
    
    def timer_callback(self):
        # getting actual positions
        x1 = self.robot_actual_pose.transform.translation.x
        y1 = self.robot_actual_pose.transform.translation.y
        z1 = self.robot_actual_pose.transform.translation.z

        # putting actual position into json format
        a_data = {
            "x": x1,
            "y": y1,
            "z": z1,
        } 

        # getting calculated positions
        x2 = self.robot_gps_pose.transform.translation.x
        y2 = self.robot_gps_pose.transform.translation.y
        z2 = self.robot_gps_pose.transform.translation.z

        # putting calculated position into json format
        c_data = {
            "x": x2,
            "y": y2,
            "z": z2,
        }
        
        # adding robot's actual & calculated positions into list
        actual_data.append(a_data)
        calc_data.append(c_data)

def write():
    # write to actual_data.json
    with open(actual_filepath, 'w') as json_file:
        json.dump(actual_data, json_file, indent=4)
    
    # write to calc_data.json
    with open(calc_filepath, 'w') as json_file:
        json.dump(calc_data, json_file, indent=4)

def main(args=None):
    rclpy.init(args=args)
    node = WriterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    write()


if __name__ == '__main__':
    main()
