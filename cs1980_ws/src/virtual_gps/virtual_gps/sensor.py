from decimal import FloatOperation
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

# wrong?
from std_msgs.msg import Float32

import math

class SensorNode(Node):

    def __init__(self):
        super().__init__('sensor')

        # this will be the ground robot name 
        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value

        print(f"robot name: {self.robot_name}")

        self.robot_pose = TransformStamped()

        # Pose variables to keep track of drone positions 
        self.x500_1_pose = TransformStamped()
        self.x500_2_pose = TransformStamped()
        self.x500_3_pose = TransformStamped()
        self.x500_4_pose = TransformStamped()

        # publisher to topic that virtual_gps will subscribe to -- msg type TBD, using float for now
        # change topic name?
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

        self.x500_4_publisher = self.create_publisher(
            Float32, 
            f'/dist/{self.robot_name}/x500_4', 
            10
        ) 

        # subscriber to '/tf' which contains robot positions
        self.x500_subscriber = self.create_subscription(TFMessage, '/tf', self.pose_callback, 10)
        self.x500_subscriber
        # timer that runs callback function every 200ms -- tf_broadcaster sending every ~20ms
        # this node has enough time to collect updates/calculate for each robot before sending 
        self.timer = self.create_timer(0.2, self.timer_callback)  # .2s -> 200ms 

    # subscriber callback function 
    def pose_callback(self, msg):

        #print("inside callback")
        # robot name in field like msg.child_frame_id or msg.transforms.child_frame_id
        for transform in msg.transforms:
            name = transform.child_frame_id

            print(f"transform name: {name}")
        
            # setting pose variable equal to msg (should be of type TransformStamped)
            if name == 'x500_1':
                self.x500_1_pose = transform
            elif name == 'x500_2':
                self.x500_2_pose = transform
            elif name == 'x500_3':
                self.x500_3_pose = transform
            elif name == 'x500_4':
                self.x500_4_pose = transform
            elif name == self.robot_name:
                self.robot_pose = transform
            else:
                print("tf message robot name error.")


    def timer_callback(self):

        # calculate distance from target robot to every flying robot
        dist1 = self.calculate_distance(self.robot_pose, self.x500_1_pose)
        dist2 = self.calculate_distance(self.robot_pose, self.x500_2_pose)
        dist3 = self.calculate_distance(self.robot_pose, self.x500_3_pose)
        dist4 = self.calculate_distance(self.robot_pose, self.x500_4_pose)

        # publish distance to each flying robot to the topic for it 
        msg = Float32() 
        msg.data = dist1
        self.x500_1_publisher.publish(msg)

        msg.data = dist2
        self.x500_2_publisher.publish(msg)

        msg.data = dist3
        self.x500_3_publisher.publish(msg)

        msg.data = dist4
        self.x500_4_publisher.publish(msg)
        

    # function to calculate distance between any 2 robots passed to it
    # args could be the self.x500_#_pose variables
    def calculate_distance(self, robot1, robot2):
        # extracting x,y,z from each pose  
        x1 = robot1.transform.translation.x
        y1 = robot1.transform.translation.y
        z1 = robot1.transform.translation.z

        x2 = robot2.transform.translation.x
        y2 = robot2.transform.translation.y
        z2 = robot2.transform.translation.z

        # using x,y,z values to calculate euclidean distance
        x_value = (x1-x2) ** 2
        y_value = (y1-y2) ** 2
        z_value = (z1-z2) ** 2

        # possibly replace these calculations with tf2 function?--probably not necessary
        euclidean_distance = math.sqrt((x_value + y_value + z_value)) 

        # here for testing purposes
        #print(euclidean_distance)
        
        return euclidean_distance


def main():

    rclpy.init()
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



