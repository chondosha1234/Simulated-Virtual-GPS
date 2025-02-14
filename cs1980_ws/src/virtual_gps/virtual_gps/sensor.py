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

        # Pose variables to keep track of drone positions 
        self.x500_0_pose = TransformStamped()
        self.x500_1_pose = TransformStamped()
        self.x500_2_pose = TransformStamped()
        self.x500_3_pose = TransformStamped()

        # publisher to topic that virtual_gps will subscribe to -- msg type TBD, using float for now
        # change topic name?
        self.x500_publisher = self.create_publisher(Float32, '/dist', 10)

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
            robot_name = transform.child_frame_id

            # here for testing purposes
            #print("Robot name: ", robot_name)
        
            # setting pose variable equal to msg (should be of type pose)
            # id names incorrect?
            if robot_name == 'x500_0':
                self.x500_0_pose = transform
            elif robot_name == 'x500_1':
                self.x500_1_pose = transform
            elif robot_name == 'x500_2':
                self.x500_2_pose = transform
            elif robot_name == 'x500_3':
                self.x500_3_pose = transform
            else:
                print("tf message robot name error.")


    def timer_callback(self):

        # calculate distance for every robot pair -- currently just 0 and 1 for testing purposes
        dist0_1 = self.calculate_distance(self.x500_0_pose, self.x500_1_pose)

        # distances for all robot pairs
        # dist0_2 = self.calculate_distance(self.x500_0_pose, self.x500_2_pose)
        # dist0_3 = self.calculate_distance(self.x500_0_pose, self.x500_3_pose)
        # dist1_2 = self.calculate_distance(self.x500_1_pose, self.x500_2_pose)
        # dist1_3 = self.calculate_distance(self.x500_1_pose, self.x500_3_pose)
        # dist2_3 = self.calculate_distance(self.x500_2_pose, self.x500_3_pose)

        # publish distances to the GPS (msg type and format TBD) -- using float for now
        # distances will be calculated and published to '/dist' for virtual_gps 
        msg = Float32() # ?
        msg.data = dist0_1
        self.x500_publisher.publish(msg)
        
        print(f"distance calculated: {dist0_1}")
        

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



