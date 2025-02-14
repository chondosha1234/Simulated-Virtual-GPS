import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage

import math

class VirtualGPSNode(Node):

    def __init__(self):
        super().__init__('virtual_gps_node')

        # Pose variables to keep track of drone positions
        self.x500_0_pose = TransformStamped()
        self.x500_1_pose = TransformStamped()
        self.x500_2_pose = TransformStamped()
        self.x500_3_pose = TransformStamped()

        # Distance variables to keep track of a distance to the target robot for each drone
        self.distance0 = Float32()
        self.distance1 = Float32()
        self.distance2 = Float32()
        self.distance3 = Float32()

        # Publisher to topic '/gps' that Error Measurement Node will subscribe to
        # Message type is TransformStamped?
        self.gps_publisher = self.create_publisher(TransformStamped, '/gps', 10)

        # Timer that runs callback function every 200ms
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Subscriber to topic '/tf' which contains drone positions
        self.x500_subscription = self.create_subscription(TFMessage, '/tf', self.pose_callback, 10)
        self.x500_subscription

        # Subscriber to topic '/dist' which contains the distance between the target robot and a specific drone
        self.sensor_subscription = self.create_subscription(Float32, '/dist', self.distance_callback, 10)
        self.sensor_subscription

    def timer_callback(self):
        target = TransformStamped()

        # Determine the Cartesian coordinates of the target robot
        target = trilateration_solver(self.x500_0_pose, self.x500_1_pose, self.x500_2_pose, self.x500_3_pose, 
                                    self.distance0, self.distance1, self.distance2, self.distance3)

        self.gps_publisher.publish(target)
    
    def pose_callback(self, msg):
        for transform in msg.transforms:
            drone_name = transform.child_frame_id

            if robot_name == 'x500_0':
                self.x500_0_pose = transform
            elif robot_name == 'x500_1':
                self.x500_1_pose = transform
            elif robot_name == 'x500_2':
                self.x500_2_pose = transform
            elif robot_name == 'x500_3':
                self.x500_3_pose = transform
            else:
                print("tf message robot name error")
    
    def distance_callback(self, msg):

    def trilateration_solver(drone0, drone1, drone2, drone3, r0, r1, r2, r3):
    
def main(args=None):
    rclpy.init(args=args)
    node = VirtualGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
