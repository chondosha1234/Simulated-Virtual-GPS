import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from std_msgs.msg import Float32
import math
import numpy as np

class VirtualGPSNode(Node):

    def __init__(self):
        super().__init__('virtual_gps_node')

        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value

        # Pose variables to keep track of drone positions
        self.x500_1_pose = TransformStamped()
        self.x500_2_pose = TransformStamped()
        self.x500_3_pose = TransformStamped()
        self.x500_4_pose = TransformStamped()

        # Distance variables to keep track of a distance to the target robot for each drone
        self.distance1 = Float32()
        self.distance2 = Float32()
        self.distance3 = Float32()
        self.distance4 = Float32()

        # Publisher to topic '/gps' that Error Measurement Node will subscribe to
        self.gps_publisher = self.create_publisher(TransformStamped, '/gps', 10)

        # Timer that runs callback function every 200ms
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Subscriber to topic '/tf' which contains drone positions
        self.x500_subscription = self.create_subscription(TFMessage, '/tf', self.pose_callback, 10)
        self.x500_subscription

        # Subscriber to topic '/dist/{self.robot_name}/x500_1'
        self.sensor_subscription_1 = self.create_subscription(Float32, f'/dist/{self.robot_name}/x500_1', self.distance_callback_1, 10)
        self.sensor_subscription_1

        # Subscriber to topic '/dist/{self.robot_name}/x500_2'
        self.sensor_subscription_2 = self.create_subscription(Float32, f'/dist/{self.robot_name}/x500_2', self.distance_callback_2, 10)
        self.sensor_subscription_2

        # Subscriber to topic '/dist/{self.robot_name}/x500_3'
        self.sensor_subscription_3 = self.create_subscription(Float32, f'/dist/{self.robot_name}/x500_3', self.distance_callback_3, 10)
        self.sensor_subscription_3

        # Subscriber to topic '/dist/{self.robot_name}/x500_4'
        self.sensor_subscription_4 = self.create_subscription(Float32, f'/dist/{self.robot_name}/x500_4', self.distance_callback_4, 10)
        self.sensor_subscription_4

    def timer_callback(self):
        target = TransformStamped()

        # Determine the Cartesian coordinates of the target robot
        target = self.trilateration_solver(self.x500_1_pose, self.x500_2_pose, self.x500_3_pose, self.x500_4_pose, 
                                    self.distance1, self.distance2, self.distance3, self.distance4)

        self.gps_publisher.publish(target)
    
    def pose_callback(self, msg):
        for transform in msg.transforms:
            drone_name = transform.child_frame_id

            if robot_name == 'x500_1':
                self.x500_1_pose = transform
            elif robot_name == 'x500_2':
                self.x500_2_pose = transform
            elif robot_name == 'x500_3':
                self.x500_3_pose = transform
            elif robot_name == 'x500_4':
                self.x500_4_pose = transform
            else:
                print("tf message robot name error")
    
    def distance_callback_1(self, msg):
        self.distance1 = msg.data

    def distance_callback_2(self, msg):
        self.distance2 = msg.data

    def distance_callback_3(self, msg):
        self.distance3 = msg.data

    def distance_callback_4(self, msg):
        self.distance4 = msg.data

    def trilateration_solver(drone0, drone1, drone2, drone3, r0, r1, r2, r3):
        # Get the cartesian coordinates of all the four drones
        x0 = drone0.transfrom.translation.x
        y0 = drone0.transfrom.translation.y
        z0 = drone0.transfrom.translation.z

        x1 = drone1.transfrom.translation.x
        y1 = drone1.transfrom.translation.y
        z1 = drone1.transfrom.translation.z

        x2 = drone2.transfrom.translation.x
        y2 = drone2.transfrom.translation.y
        z2 = drone2.transfrom.translation.z

        x3 = drone3.transfrom.translation.x
        y3 = drone3.transfrom.translation.y
        z3 = drone3.transfrom.translation.z

        p0 = np.array([x0, y0, z0])
        p1 = np.array([x1, y1, z1])
        p2 = np.aaray([x2, y2, z2])

        # Perform a world frame transform to simplify calculation
        # p0 is the new origin, p1 is on the x axis, and p2 is on the x-y plane
        # ex, ey, and ez are unit vectors along x axis, y axis, and z axis respectively
        # In this world frame, p0 (0, 0, 0), p1 (d, 0, 0), p2 (i, j, 0)
        ex = (p1 - p0) / (np.linalg.norm(p1 - p0))
        i = np.dot(ex, p2 - p0)
        ey = (p2 - p0 - i * ex) / (np.linalg.norm(p2 - p0 - i * ex))
        ez = np.cross(ex, ey)
        d = np.linale.norm(p1 - p0)
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

        if pow((x3 - x), 2) + pow((y3 - y), 2) + pow((z3 - z_a), 2) == pow(r3, 2):
            z = z_a
        else:
            z = z_b
        
        target = TransformStamped()
        target.transform.translation.x = x
        target.transform.translation.y = y
        target.transform.translation.z = z

        return target

    
def main(args=None):
    rclpy.init(args=args)
    node = VirtualGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
