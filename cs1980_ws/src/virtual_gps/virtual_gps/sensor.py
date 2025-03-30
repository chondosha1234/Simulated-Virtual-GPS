from decimal import FloatOperation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32

import math

class SensorNode(Node):

    def __init__(self):
        super().__init__('sensor')

        # this will be the ground robot name 
        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value

        self.get_logger().info(f'robot name in sensor: {self.robot_name}')

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.robot_pose = TransformStamped()

        # Pose variables to keep track of drone positions 
        self.x500_0_pose = TransformStamped()
        self.x500_1_pose = TransformStamped()
        self.x500_2_pose = TransformStamped()
        self.x500_3_pose = TransformStamped()

        # publisher to topic that virtual_gps will subscribe to -- msg type TBD, using float for now
        # change topic name?
        self.x500_0_publisher = self.create_publisher(
            Float32, 
            f'/dist/{self.robot_name}/x500_0', 
            10
        ) 

        self.x500_1_publisher = self.create_publisher(
            Float32, 
            f'/dist/{self.robot_name}/x500_1', 
            10
        ) 

        self.x500_2_publisher = self.create_publisher(
            Float32, 
            f'/dist/{self.robot_name}/x500_2', 
            10
        ) 

        self.x500_3_publisher = self.create_publisher(
            Float32, 
            f'/dist/{self.robot_name}/x500_3', 
            10
        ) 

        # subscriber to '/tf' which contains robot positions
        #self.x500_subscriber = self.create_subscription(TFMessage, '/tf', self.pose_callback, 10)
        #self.x500_subscriber

        self.x500_0_sub = self.create_subscription(TransformStamped, '/model/x500_0/pose', self.x500_0_sub_callback, 1)
        self.x500_0_sub

        self.x500_1_sub = self.create_subscription(TransformStamped, '/model/x500_1/pose', self.x500_1_sub_callback, 1)
        self.x500_1_sub

        self.x500_2_sub = self.create_subscription(TransformStamped, '/model/x500_2/pose', self.x500_2_sub_callback, 1)
        self.x500_2_sub

        self.x500_3_sub = self.create_subscription(TransformStamped, '/model/x500_3/pose', self.x500_3_sub_callback, 1)
        self.x500_3_sub

        self.raspimouse_sub = self.create_subscription(TransformStamped, f'/model/{self.robot_name}/pose', self.raspimouse_pose_callback, 1)
        self.raspimouse_sub
        #self.raspimouse_subscriber = self.create_subscription(TransformStamped, f'/model/{self.robot_name}/pose', self.mouse_callback, 10)
        #self.raspimouse_subscriber

        # timer that runs callback function every 200ms -- tf_broadcaster sending every ~20ms
        # this node has enough time to collect updates/calculate for each robot before sending 
        self.timer = self.create_timer(0.2, self.timer_callback)  # .2s -> 200ms 


    def x500_0_sub_callback(self, msg):
        self.x500_0_pose = msg

    def x500_1_sub_callback(self, msg):
        self.x500_1_pose = msg

    def x500_2_sub_callback(self, msg):
        self.x500_2_pose = msg

    def x500_3_sub_callback(self, msg):
        self.x500_3_pose = msg

    def raspimouse_pose_callback(self, msg):
        if msg.child_frame_id == 'raspimouse':
            self.robot_pose = msg


    def timer_callback(self):

        #self.get_logger().info('sensor timer callback')

        # calculate distance from target robot to every flying robot
        dist0 = self.calculate_distance(self.robot_pose, self.x500_0_pose)
        dist1 = self.calculate_distance(self.robot_pose, self.x500_1_pose)
        dist2 = self.calculate_distance(self.robot_pose, self.x500_2_pose)
        dist3 = self.calculate_distance(self.robot_pose, self.x500_3_pose)

        # publish distance to each flying robot to the topic for it 
        msg0 = Float32() 
        msg0.data = dist0
        self.x500_0_publisher.publish(msg0)

        msg1 = Float32() 
        msg1.data = dist1
        self.x500_1_publisher.publish(msg1)

        msg2 = Float32()
        msg2.data = dist2
        self.x500_2_publisher.publish(msg2)

        msg3 = Float32()
        msg3.data = dist3
        self.x500_3_publisher.publish(msg3)
        

    # function to calculate distance between any 2 robots passed to it
    # args could be the self.x500_#_pose variables
    def calculate_distance(self, robot1, robot2):
        # extracting x,y,z from each pose  
        x1 = robot1.transform.translation.x
        y1 = robot1.transform.translation.y
        z1 = robot1.transform.translation.z

        #self.get_logger().info(f'mouse x: {x1}')
        #self.get_logger().info(f'mouse y: {y1}')
        #self.get_logger().info(f'mouse z: {z1}')

        x2 = robot2.transform.translation.x
        y2 = robot2.transform.translation.y
        z2 = robot2.transform.translation.z

        #self.get_logger().info(f'drone x: {x2}')
        #self.get_logger().info(f'drone y: {y2}')
        #self.get_logger().info(f'drone z: {z2}')

        # using x,y,z values to calculate euclidean distance
        x_value = (x1-x2) ** 2
        y_value = (y1-y2) ** 2
        z_value = (z1-z2) ** 2

        # possibly replace these calculations with tf2 function?--probably not necessary
        euclidean_distance = math.sqrt((x_value + y_value + z_value)) 

        # here for testing purposes
        #self.get_logger().info(f'sensor distance calc: {euclidean_distance}')
        
        return euclidean_distance


def main():

    rclpy.init()
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



