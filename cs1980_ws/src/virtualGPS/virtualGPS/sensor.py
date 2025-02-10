import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped

class SensorNode(Node):

    def __init__(self):
        super().__init__('sensor')

        # add a publisher to some topic that GPS will subscribe too -- message type could be something like Float? or maybe a message type that has String and float?

        # add subsriber for topic '/tf' - it will hold all robot positions 
        # the robot name will be in field like msg.child_frame_id or maybe msg.transforms.child_frame_id 

        # add Pose variables to keep track of drone positions 
        self.x500_0_pose = Pose()

        # also add a timer, and when the timer fire it runs a callback function. 
        # this callback can then use the calculate_distance function to do all the distances and publish along the next topic to send to virtual_gps node 
        # the timer callback could be like every 100 or 200 ms, and the tf_broadcaster will be sending every 20 ms lets say 
        # that way this node has enough time to collect updates for each robot and calculate new distance before sending it on 
        
        self.timer = self.create_timer(0.2, self.timer_callback)  # .2 sec = 200 ms 

    # subscriber callback function 
    def pose_callback(self, msg):
        # the message should be type Pose, so can set it to the pose variable for each robot declared in this file ?
        # also, remember to check the msg 'robot name' and assign it to the appropriate variable from __init__
        return 0


    def timer_callback(self):
        # calculate distance for every robot pair 
        # i.e.  0,1  0,2  0,3  1,2  1,3  2,3 etc. 
        # and publish to the GPS (msg type and format TBD)
        return 0

    
    # function to calculate distance between any 2 robots passed to it
    def calculate_distance(self, robot1, robot2):

        # args could be the self.x500_#_pose variables
        # then extract the x,y,z from each and do euclidean distance calculation 

        # ** there might be a tf2 function that auto calculates distances? not sure **
        
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


