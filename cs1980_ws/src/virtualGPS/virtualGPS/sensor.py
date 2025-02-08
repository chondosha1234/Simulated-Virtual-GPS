import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose

class SensorNode(Node):

    def __init__(self):
        super().__init__('sensor')

        # add a publisher to some topic that GPS will subscribe too -- message type could be something like Float? or maybe a message type that has String and float?

        # add subscribers to topics like /model/x500_0/pose

        # add Pose variables to keep track of drone positions 
        self.x500_0_pose = Pose()


    # subscriber callback function 
    def x500_0_pose_callback(self, msg):
        # the message should be type Pose, so can set it to the pose variable declared in this file ?
        return 0

    
    # function to calculate distance between any 2 robots passed to it
    def calculate_distance(self, robot1, robot2):

        # args could be the self.x500_#_pose variables
        # then extract the x,y,z from each and do euclidean distance calculation 
        
        # return a distance and/or publish it 
        return 0


def main():

    rclpy.init()
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
