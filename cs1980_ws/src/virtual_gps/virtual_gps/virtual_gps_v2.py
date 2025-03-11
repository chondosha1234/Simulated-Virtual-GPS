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

        self.get_logger().info(f'robot name in gps: {self.robot_name}')

        # will use to determine how many/which drones exist
        # subscriber will only be made for existing drones
        self.pose_list = [0] * 4
        self.num_drones = 0

        # Publisher to topic '/gps' that Error Measurement Node will subscribe to
        self.gps_publisher = self.create_publisher(TransformStamped, f'/{self.robot_name}/gps', 10)

        # not using for now because maybe not necessary? 
        # e.g. 3 drones 1 buffer, 2 drones 2 buffers, 1 drone 3 buffers
        # self.buffer_num = 4 - self.num_drones
        
        self.pose_buffer = []
        self.distance_buffer = []

        # boolean to check if buffer has been filled to ensure there is enough data to do calculations 
        # shouldn't need both of these but may be good to have for testing purposes
        self.filled = False
        self.filled2 = False

        # Subscriber to topic '/tf' which contains drone positions
        self.x500_subscription = self.create_subscription(TFMessage, '/tf', self.pose_callback, 10)
        self.x500_subscription

        # making subscribers only if drone exists; cause errors otherwise?
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

        # timer only starts once buffer is filled
        # e.g. only one drone, timer_callback won't be called until 3 previous distances recorded
        # Timer that runs callback function every 200ms
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        #self.get_logger().info('gps timer callback')
        if self.filled == True and self.filled2 == True:

            target = TransformStamped()

            # Determine the Cartesian coordinates of the target robot
            target = self.trilateration_solver(self.pose_buffer[0], self.pose_buffer[1], self.pose_buffer[2], self.pose_buffer[3], 
                                        self.distance_buffer[0], self.distance_buffer[1], self.distance_buffer[2], self.distance_buffer[3])
            self.gps_publisher.publish(target)


    # if the drone exists, save its position and record its existence
    def pose_callback(self, msg):
        for transform in msg.transforms:
            name = transform.child_frame_id

            if name == 'x500_1':
                self.pose_filled = self.buffer(self.pose_buffer, transform)
                self.pose_list[0] = 1
            elif name == 'x500_2':
                # self.x500_2_pose = transform
                self.pose_filled = self.buffer(self.pose_buffer, transform)
                self.pose_list[1] = 1
            elif name == 'x500_3':
                # self.x500_3_pose = transform
                self.pose_filled = self.buffer(self.pose_buffer, transform)
                self.pose_list[2] = 1
            elif name == 'x500_4':
                # self.x500_4_pose = transform
                self.pose_filled = self.buffer(self.pose_buffer, transform)
                self.pose_list[3] = 1
        
        # count the number of drones in simulation
        i = 0
        for drone in self.pose_list:
            if drone == 1:
                i += 1
        self.num_drones = i
    

    def buffer(self, buffer, value):
        filled = False
        if len(buffer) < 4:
            buffer.append(value)
            if len(buffer) == 4:
                filled = True
        elif len(buffer) == 4:
            buffer.pop(0)
            buffer.append(value)
            filled = True
        #self.get_logger().info(f'buffer: {buffer}')
        return filled

    def distance_callback_1(self, msg):
        self.dist_filled = self.buffer(self.distance_buffer, msg.data)

    def distance_callback_2(self, msg):
        self.dist_filled = self.buffer(self.distance_buffer, msg.data)

    def distance_callback_3(self, msg):
        self.dist_filled = self.buffer(self.distance_buffer, msg.data)

    def distance_callback_4(self, msg):
        self.dist_filled = self.buffer(self.distance_buffer, msg.data)

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

    
def main(args=None):
    rclpy.init(args=args)
    node = VirtualGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

