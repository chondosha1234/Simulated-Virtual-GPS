import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

class WheelNode(Node):

    def __init__(self):
        super().__init__('wheel_node')

        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value

        self.left_wheel = TransformStamped()
        self.right_wheel = TransformStamped()

        self.wheel_subscription = self.create_subscription(TFMessage, '/tf', self.pose_callback, 10)
        self.wheel_subscription

        self.left_wheel_pub = self.create_publisher(TransformStamped, f'/{self.robot_name}/left_wheel', 10)

        self.right_wheel_pub = self.create_publisher(TransformStamped, f'/{self.robot_name}/right_wheel', 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def pose_callback(self, msg):

        for transform in msg.transforms:
            name = transform.child_frame_id

            if name == 'left_wheel':
                self.left_wheel = transform
            elif name == 'right_weel':
                self.right_wheel = transform

    
    def timer_callback(self):
        # publish current wheel positions to their topics
        self.left_wheel_pub.publish(self.left_wheel)
        self.right_wheel_pub.publish(self.right_wheel)


def main(args=None):
    rclpy.init(args=args)
    node = WheelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


