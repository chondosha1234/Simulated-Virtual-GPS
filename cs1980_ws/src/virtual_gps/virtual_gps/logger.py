import subprocess
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from std_msgs.msg import Float32
import math
import numpy as np

import logging

class LoggerNode(Node):

    def __init__(self):
        super().__init__('logging_node')

        logging.basicConfig(filename='results.log', level=logging.DEBUG, filemode='a', format='%(asctime)s - %(message)s', force=True)

        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value

        # variable declarations--modified in callback functions
        self.robot_text = f"robot name in gps: {self.robot_name}"

        self.x500_0_text = "x500_0 is not in simulation"
        self.x500_1_text = "x500_1 is not in simulation"
        self.x500_2_text = "x500_2 is not in simulation"
        self.x500_3_text = "x500_3 is not in simulation"

        self.dist0_text = ""
        self.dist1_text = ""
        self.dist2_text = ""
        self.dist3_text = ""

        self.gps_value = TransformStamped()
        self.gps_text = ""

        self.error_value = 0.0
        self.error_text = ""

        # Subscriber to topic '/tf' which contains drone positions
        #self.x500_subscription = self.create_subscription(TFMessage, '/tf', self.pose_callback, 10)
        #self.x500_subscription

        # Subscriber to topic '/dist/{self.robot_name}/x500_0'
        self.sensor_subscription_0 = self.create_subscription(Float32, f'/dist/{self.robot_name}/x500_0', self.distance_callback_0, 10)
        self.sensor_subscription_0

        # Subscriber to topic '/dist/{self.robot_name}/x500_1'
        self.sensor_subscription_1 = self.create_subscription(Float32, f'/dist/{self.robot_name}/x500_1', self.distance_callback_1, 10)
        self.sensor_subscription_1

        # Subscriber to topic '/dist/{self.robot_name}/x500_2'
        self.sensor_subscription_2 = self.create_subscription(Float32, f'/dist/{self.robot_name}/x500_2', self.distance_callback_2, 10)
        self.sensor_subscription_2

        # Subscriber to topic '/dist/{self.robot_name}/x500_3'
        self.sensor_subscription_3 = self.create_subscription(Float32, f'/dist/{self.robot_name}/x500_3', self.distance_callback_3, 10)
        self.sensor_subscription_3

        self.x500_0_sub = self.create_subscription(TransformStamped, '/model/x500_0/pose', self.x500_0_sub_callback, 10)
        self.x500_0_sub

        self.x500_1_sub = self.create_subscription(TransformStamped, '/model/x500_1/pose', self.x500_1_sub_callback, 10)
        self.x500_1_sub

        self.x500_2_sub = self.create_subscription(TransformStamped, '/model/x500_2/pose', self.x500_2_sub_callback, 10)
        self.x500_2_sub

        self.x500_3_sub = self.create_subscription(TransformStamped, '/model/x500_3/pose', self.x500_3_sub_callback, 10)
        self.x500_3_sub

        self.gps_subscription = self.create_subscription(TransformStamped, f'/{self.robot_name}/gps', self.gps_callback, 10)
        self.gps_subscription

        self.error_subscription = self.create_subscription(Float32, f'/{self.robot_name}/error_measure', self.error_callback, 10)
        self.error_subscription

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        text = f"{self.robot_text}{self.x500_0_text}{self.x500_1_text}{self.x500_2_text}{self.x500_3_text}{self.dist0_text}{self.dist1_text}{self.dist2_text}{self.dist3_text}{self.gps_text}{self.error_text}\n"
        logging.info(f'{self.robot_text}')
        logging.info(f'{self.x500_0_text}')
        logging.info(f'{self.x500_1_text}')
        logging.info(f'{self.x500_2_text}')
        logging.info(f'{self.x500_3_text}')
        logging.info(f'{self.dist0_text}')
        logging.info(f'{self.dist1_text}')
        logging.info(f'{self.dist2_text}')
        logging.info(f'{self.dist3_text}')
        logging.info(f'{self.gps_text}')
        logging.info(f'{self.error_text}')
        self.get_logger().info(text)


    def distance_callback_0(self, msg):
        self.dist0_text = f"distance value for x500_0: {msg.data}"

    def distance_callback_1(self, msg):
        self.dist1_text = f"distance value for x500_1: {msg.data}"

    def distance_callback_2(self, msg):
        self.dist2_text = f"distance value for x500_2: {msg.data}"

    def distance_callback_3(self, msg):
        self.dist3_text = f"distance value for x500_3: {msg.data}\n\n"

    def x500_0_sub_callback(self, msg):
        self.x500_0_text = "x500_0 is in simulation\n"

    def x500_1_sub_callback(self, msg):
        self.x500_1_text = "x500_1 is in simulation\n"

    def x500_2_sub_callback(self, msg):
        self.x500_2_text = "x500_2 is in simulation\n"

    def x500_3_sub_callback(self, msg):
        self.x500_3_text = "x500_3 is in simulation\n\n"
    
    def gps_callback(self, msg):
        self.gps_text = f"gps location-- x: {msg.transform.translation.x}, y: {msg.transform.translation.y}, z: {msg.transform.translation.z}\n\n"

    def error_callback(self, msg):
        self.error_text = f"error measurement: {msg.data}"
    
def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
