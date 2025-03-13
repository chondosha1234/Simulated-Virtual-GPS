
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose

from tf2_ros import TransformBroadcaster

class Tf2PositionBroadcaster(Node):
    
    def __init__(self):

        super().__init__('tf2_broadcaster')

        #declare and acquire robot name parameter (parameters can be set for each Node in launch file, each robot will launch their own version)
        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value

        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.pose_sub = self.create_subscription(
                Pose,
                f'/model/{self.robot_name}/pose',   # each robot will have its own topic 
                self.pose_callback,
                qos_profile
        )


    def pose_callback(self, msg):
        # pass the msg data in Pose to broadcast
        self.broadcast_tf(msg)
        

    def broadcast_tf(self, pose):

        t = TransformStamped()   # create new message type to send out 
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = f"{self.robot_name}"

        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main():

    rclpy.init()
    node = Tf2PositionBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



