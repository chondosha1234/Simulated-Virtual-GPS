import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from std_msgs.msg import Float32

import tf_transformations
import copy 
import math
import numpy as np
from scipy import linalg

class VirtualGPSNode(Node):

    def __init__(self):
        super().__init__('virtual_gps_node')

        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value
        self.get_logger().info(f'robot name in gps: {self.robot_name}')

        self.num_drones = self.declare_parameter('num_drones', 1).get_parameter_value().integer_value

        # used to determine how many/which drones exist
        #self.pose_list = [0] * 4

        # Publisher to topic '/gps' that Error Measurement Node will subscribe to
        self.gps_publisher = self.create_publisher(TransformStamped, f'/{self.robot_name}/gps', 10)
        
        # placeholder for pose values
        self.pose0 = TransformStamped()
        self.pose1 = TransformStamped()
        self.pose2 = TransformStamped()
        self.pose3 = TransformStamped()

        # placeholder for dist valuea
        self.dist0 = 0.0
        self.dist1 = 0.0
        self.dist2 = 0.0
        self.dist3 = 0.0

        # the buffer--list of tuples which hold pose and dist values
        self.buffer_list = []

        # tracks whether buffer is full
        self.filled = False

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

        # Timer that runs callback function every 200ms
        timer_period = 0.25
        self.gps_timer = self.create_timer(timer_period, self.gps_timer_callback)

        # timer that runs callback function every 250ms
        timer_period2 = 0.25
        self.buffer_timer = self.create_timer(timer_period2, self.buffer_timer_callback)


    # timer only starts once buffer is filled
    # e.g. if only one drone, timer_callback won't be called until 3 previous values and current values in buffer
    def gps_timer_callback(self):

        #self.get_logger().info('gps timer callback')
        if self.filled == True:
            target = TransformStamped()

            # Determine the Cartesian coordinates of the target robot
            target = self.trilateration_solver(self.buffer_list[0][0], self.buffer_list[1][0], self.buffer_list[2][0], self.buffer_list[3][0], 
                                        self.buffer_list[0][1], self.buffer_list[1][1], self.buffer_list[2][1], self.buffer_list[3][1])

            if target != None:
                self.gps_publisher.publish(target)


    # counts number of drones and adds drone data to buffer
    def buffer_timer_callback(self):

        # given number of drones in simulation, add drone data to buffer
        # self.dist values from distance_callback functions
        if self.num_drones == 1:
            self.buffer(self.buffer_list, self.pose0, self.dist0)

        elif self.num_drones == 2:
            self.buffer(self.buffer_list, self.pose0, self.dist0)
            self.buffer(self.buffer_list, self.pose1, self.dist1)

        elif self.num_drones == 4:    
            self.buffer(self.buffer_list, self.pose0, self.dist0)
            self.buffer(self.buffer_list, self.pose1, self.dist1)
            self.buffer(self.buffer_list, self.pose2, self.dist2)
            self.buffer(self.buffer_list, self.pose3, self.dist3)


    def buffer(self, buffer, pose, distance):

        # copy the object, otherwise it will be updated in buffer when self.pose gets changed 
        pose_copy = copy.deepcopy(pose)
        pose_dist_tuple = (pose_copy, distance)

        # if buffer is not filled, add tuple to buffer
        if len(buffer) < 4:
            buffer.append(pose_dist_tuple)

        # if buffer is filled, dump buffer before adding tuple--number of elements dumped dependent on number of drones in sim
        # i.e. dumps 1 if 1 drone, 2 if 2 drones, 4 if 4 drones
        elif len(buffer) == 4:
            #for i in range(1, self.num_drones):
            #    buffer.pop(0)
            #self.get_logger().info(f'replacing one buffer spot')
            buffer.pop(0)
            buffer.append(pose_dist_tuple)
        
        self.filled = False
        # set filled to true if buffer is filled after adding tuple to it
        if len(buffer) == 4:
            self.filled = True

    def distance_callback_0(self, msg):
        self.dist0 = msg.data

    def distance_callback_1(self, msg):
        self.dist1 = msg.data

    def distance_callback_2(self, msg):
        self.dist2 = msg.data

    def distance_callback_3(self, msg):
        self.dist3 = msg.data

    def x500_0_sub_callback(self, msg):
        self.pose0 = msg

    def x500_1_sub_callback(self, msg):
        self.pose1 = msg

    def x500_2_sub_callback(self, msg):
        self.pose2 = msg

    def x500_3_sub_callback(self, msg):
        self.pose3 = msg


    def trilateration_solver(self, drone0, drone1, drone2, drone3, r0, r1, r2, r3):
        # Get the cartesian coordinates of all the four drones
        x0 = drone0.transform.translation.x
        y0 = drone0.transform.translation.y
        z0 = drone0.transform.translation.z

        x1 = drone1.transform.translation.x
        y1 = drone1.transform.translation.y
        z1 = drone1.transform.translation.z

        x2 = drone2.transform.translation.x
        y2 = drone2.transform.translation.y
        z2 = drone2.transform.translation.z

        x3 = drone3.transform.translation.x
        y3 = drone3.transform.translation.y
        z3 = drone3.transform.translation.z

        A = np.array(
            [
                [x1-x0, y1-y0, z1-z0],
                [x2-x0, y2-y0, z2-z0],
                [x3-x0, y3-y0, z3-z0],
            ]
        )

        b = np.array(
            [
                0.5*(x1**2 - x0**2 + y1**2 - y0**2 + z1**2 - z0**2 + r0**2 - r1**2),
                0.5*(x2**2 - x0**2 + y2**2 - y0**2 + z2**2 - z0**2 + r0**2 - r2**2),
                0.5*(x3**2 - x0**2 + y3**2 - y0**2 + z3**2 - z0**2 + r0**2 - r3**2),
            ]
        )

        # Calculate the determinant of matrix A
        A_det = linalg.det(A)
        # If it is 0, the matrix is linearly dependent, which means it is not invertible
        if A_det == 0:
            return None

        # Solve the system of equations
        A_inv = linalg.inv(A)
        x = A_inv @ b

        # Create a TransformStamped message
        target = TransformStamped()
        target.transform.translation.x = x[0]
        target.transform.translation.y = x[1]
        target.transform.translation.z = x[2]

        return target

    
def main(args=None):
    rclpy.init(args=args)
    node = VirtualGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
