import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from std_msgs.msg import Float32
import math
import numpy as np
from scipy import linalg

class VirtualGPSNode(Node):

    def __init__(self):
        super().__init__('virtual_gps_node')

        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value

        self.get_logger().info(f'robot name in gps: {self.robot_name}')

        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Pose variables to keep track of drone positions
        self.x500_0_pose = TransformStamped()
        self.x500_1_pose = TransformStamped()
        self.x500_2_pose = TransformStamped()
        self.x500_3_pose = TransformStamped()

        # Distance variables to keep track of a distance to the target robot for each drone
        self.distance0 = 0.0
        self.distance1 = 0.0
        self.distance2 = 0.0
        self.distance3 = 0.0

        # Publisher to topic '/gps' that Error Measurement Node will subscribe to
        self.gps_publisher = self.create_publisher(TransformStamped, f'/{self.robot_name}/gps', 10)

        # Timer that runs callback function every 200ms
        timer_period = 0.25
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Subscriber to topic '/tf' which contains drone positions
        #self.x500_subscription = self.create_subscription(TFMessage, '/tf', self.pose_callback, qos_profile)
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


    def timer_callback(self):
        #self.get_logger().info('gps timer callback')

        target = TransformStamped()

        # Determine the Cartesian coordinates of the target robot
        target = self.trilateration_solver(self.x500_0_pose, self.x500_1_pose, self.x500_2_pose, self.x500_3_pose, 
                                    self.distance0, self.distance1, self.distance2, self.distance3)

        if target != None:
            self.gps_publisher.publish(target)


    def distance_callback_0(self, msg):
        self.distance0 = msg.data

    def distance_callback_1(self, msg):
        self.distance1 = msg.data

    def distance_callback_2(self, msg):
        self.distance2 = msg.data

    def distance_callback_3(self, msg):
        self.distance3 = msg.data

    def x500_0_sub_callback(self, msg):
        self.x500_0_pose = msg

    def x500_1_sub_callback(self, msg):
        self.x500_1_pose = msg

    def x500_2_sub_callback(self, msg):
        self.x500_2_pose = msg

    def x500_3_sub_callback(self, msg):
        self.x500_3_pose = msg
    
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
    
    '''
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

        p0 = np.array([x0, y0, z0])
        p1 = np.array([x1, y1, z1])
        p2 = np.array([x2, y2, z2])

        # Perform a world frame transform to simplify calculation
        # p0 is the new origin, p1 is on the x axis, and p2 is on the x-y plane
        # ex, ey, and ez are unit vectors along x axis, y axis, and z axis respectively
        # In this world frame, p0 (0, 0, 0), p1 (d, 0, 0), p2 (i, j, 0)
        ex = (p1 - p0) / (np.linalg.norm(p1 - p0))
        i = np.dot(ex, p2 - p0)
        ey = (p2 - p0 - i * ex) / (np.linalg.norm(p2 - p0 - i * ex))
        ez = np.cross(ex, ey)
        d = np.linalg.norm(p1 - p0)
        j = np.dot(ey, p2 - p0)

        # Plug and chug using above values
        x = (pow(r0, 2) - pow(r1, 2) + pow(d, 2)) / (2 * d)
        y = (pow(r0, 2) - pow(r2, 2) + pow(i, 2) + pow(j ,2) - 2 * i * x) / (2 * j)

        # There will be two possible solutions for z
        z_a = np.sqrt(pow(r0, 2) - pow(x, 2) - pow(y, 2))
        z_b = -1 * np.sqrt(pow(r0, 2) - pow(x, 2) - pow(y, 2))

        # triPt is an array with the cartesian coordinate in the original world frame (before this transformation) of the trilateration point
        triPt_a = p0 + x * ex + y * ey + z_a * ez
        triPt_b = p0 + x * ex + y * ey + z_b * ez

        d_a = pow((x3 - x), 2) + pow((y3 - y), 2) + pow((z3 - z_a), 2) - pow(r3, 2)
        d_b = pow((x3 - x), 2) + pow((y3 - y), 2) + pow((z3 - z_b), 2) - pow(r3, 2)
        if d_a < d_b:
            triPt = triPt_a
        else:
            triPt = triPt_b
        
        target = TransformStamped()
        target.transform.translation.x = triPt[0]
        target.transform.translation.y = triPt[1]
        target.transform.translation.z = triPt[2]

        return target
    '''
    
def main(args=None):
    rclpy.init(args=args)
    node = VirtualGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
