import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransfromStamped
from std_msgs.msg import Float32

import math
import numpy as np

class OrientationNode(Node):

    def __init__(self):
        super().__init__('orientation_node')

        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value
        self.get_logger().info(f'robot name in orientation detection: {self.robot_name}')

        self.init_wheels = np.empty(2, dtype=object)
        self.curr_wheels = np.empty(2, dtype=object)

        self.orientation_publisher = self.create_publisher(Float32, f'/{self.robot_name}/gps', 10)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.left_wheel_subscription = self.create_subscription(TransfromStamped, "/raspimouse_left_wheel", self.left_wheel_callback, 10)
        self.left_wheel_subscription

        self.right_wheel_subscription = self.create_subscription(TransfromStamped, "/raspimouse_right_wheel", self.right_wheel_callback, 10)
        self.right_wheel_subscription

    def timer_callback(self):
        orient_change = 0.0

        orient_change = self.calculate_orient_change(self.init_wheels, self.curr_wheels)

        self.orientation_publisher.publish(orient_change)
    
    def left_wheel_callback(self, msg):
        if self.init_wheels[0] == None:
            self.init_wheels[0] = msg.data

        self.curr_wheels[0] = msg.data

    def right_wheel_callback(self, msg):
        if self.init_wheels[1] == None:
            self.init_wheels[1] == msg.data
        
        self.curr_wheels[1] == msg.data

    def calculate_orient_change(self, init_wheels, curr_wheels):
        if None in init_wheels:
            return 0

        init_left_x = init_wheels[0].transform.translation.x
        init_left_y = init_wheels[0].transform.translation.y
        init_left = [init_left_x, init_left_y]

        init_right_x = init_wheels[1].transform.translation.x
        init_right_y = init_wheels[1].transform.translation.y
        init_right = [init_right_x, init_right_y]

        v0 = [j-i for i, j in zip(init_left, init_right)]

        curr_left_x = curr_wheels[0].transform.translation.x
        curr_left_y = curr_wheels[0].transform.translation.y
        curr_left = [curr_left_x, curr_left_y]

        curr_right_x = curr_wheels[1].transform.translation.x
        curr_right_y = curr_wheels[1].transform.translation.y
        curr_right = [curr_right_x, curr_right_y]

        v1 = [j-i for i, j in zip(curr_left, curr_right)]

        dot_product = sum(i*j for i, j in zip(v0, v1))
        norm_v0 = math.sqrt(sum(i**2 for i in v0))
        norm_v1 = math.sqrt(sum(i**2 for i in v1))
        cos_theta = dot_product / (norm_v0 * norm_v1)
        angle_rad = math.acos(cos_theta)
        angle_deg = math.degrees(angle_rad)
        return angle_deg


def main(args=None):
    rclpy.init(args=args)
    node = OrientationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()