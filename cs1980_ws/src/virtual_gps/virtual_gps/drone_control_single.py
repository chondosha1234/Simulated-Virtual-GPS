import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped, PoseStamped, TransformStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOLLocal, CommandTOL, ParamSetV2
from sensor_msgs.msg import NavSatFix
import time

class DroneControl(Node):

    def __init__(self):

        super().__init__('drone')

        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value

        qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
        )

        self.state = None
        self.target_pose = PoseStamped()
        self.current_pose = PoseStamped()
        self.raspimouse_pose = PoseStamped()

        self.local_pos_publisher = self.create_publisher(PoseStamped, f'/{self.robot_name}/setpoint_position/local', 10)

        self.local_pos_subscriber = self.create_subscription(
            PoseStamped, 
            f'/{self.robot_name}/local_position/pose', 
            self.local_pos_callback, 
            qos_profile
        )
        self.local_pos_subscriber
        
        self.state_subscriber = self.create_subscription(
            State, 
            f'/{self.robot_name}/state', 
            self.state_callback, 
            10
        )
        self.state_subscriber

        self.gps_sub = self.create_subscription(
                NavSatFix,
                f'/{self.robot_name}/global_position/global',
                self.gps_fix_callback,
                qos_profile
        )
        self.gps_sub

        self.mouse_gps_sub = self.create_subscription(
            TransformStamped,
            f'/raspimouse/gps',
            self.mouse_gps_callback,
            qos_profile
        )
        self.mouse_gps_sub

        self.arming_client = self.create_client(CommandBool, f'/{self.robot_name}/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, f'/{self.robot_name}/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, f'/{self.robot_name}/cmd/takeoff')
        self.land_client = self.create_client(CommandTOLLocal, f'/{self.robot_name}/cmd/land_local')
        self.param_client = self.create_client(ParamSetV2, f'/{self.robot_name}/param/set')

        self.altitude = 0.0
        self.start_altitude = 0.0
        self.target_altitude = 0.0
        self.lat = 0.0
        self.long = 0.0


    def wait_for_service(self, client):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for service {client.srv_name} ...')


    def state_callback(self, msg):
        #self.get_logger().info("inside state callback")
        self.state = msg


    def local_pos_callback(self, msg):
        self.current_pose = msg


    def gps_fix_callback(self, msg):
        #self.get_logger().info(f"GPS altitude: {msg.altitude}")
        if self.start_altitude == 0.0:
            self.start_altitude = msg.altitude
        self.altitude = msg.altitude

        if self.lat == 0.0:
            self.lat = msg.latitude

        if self.long == 0.0:
            self.long = msg.longitude

    def mouse_gps_callback(self, msg):
        self.raspimouse_pose = msg

        
    def go_to_position(self, x, y, z):
        #self.get_logger().info(f"going to point {x}, {y}, {z}")

        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z

        self.local_pos_publisher.publish(self.target_pose)


    def wait_for_position(self, x, y, z):

        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z

        while abs(self.current_pose.pose.position.x - self.target_pose.pose.position.x) > 1.5 or abs(self.current_pose.pose.position.y - self.target_pose.pose.position.y) > 1.5:
            self.go_to_position(x, y, z)
            if self.state.mode != "OFFBOARD":
                self.get_logger().info("switch to offboard again")
                self.set_mode("OFFBOARD")
            time.sleep(0.1)
            rclpy.spin_once(self)

    
    def takeoff_drone(self, alt):

        self.wait_for_service(self.takeoff_client)
        self.target_altitude = self.altitude + alt

        request = CommandTOL.Request()
        request.min_pitch = 0.0
        request.yaw = 0.0
        request.latitude = self.lat
        request.longitude = self.long
        request.altitude = self.target_altitude

        self.takeoff_client.call_async(request)


    def wait_for_altitude(self, timeout=30):
        
        start_time = time.time()
        while time.time() - start_time < timeout:

            if self.altitude >= self.target_altitude:
                self.get_logger().info('Reached target altitude.')
                return True
            time.sleep(0.1)
            rclpy.spin_once(self)
            
            #self.get_logger().info(f'altitude: {self.target_altitude}    curr: {self.altitude}')

        self.get_logger().warn('Timeout waiting for altitude.')
        return False


    def arm_drone(self):
        self.get_logger().info("starting arming")
        self.wait_for_service(self.arming_client)
        request = CommandBool.Request()
        request.value = True
        #self.arming_client.call_async(request)
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


    def wait_for_arming(self):
        while not self.state.armed:
            time.sleep(0.1)
            rclpy.spin_once(self)


    def set_mode(self, mode):
        self.wait_for_service(self.set_mode_client)
        request = SetMode.Request()
        request.custom_mode = mode
        self.get_logger().info(f"Changing mode to: {mode}")
        self.set_mode_client.call_async(request)

    
    def set_param(self, param_name, value):
        self.wait_for_service(self.param_client)
        request = ParamSetV2.Request()
        request.param_id = param_name
        request.value.double_value = float(value)
        #self.get_logger().info(f"Setting param {param_name} to {value}")
        future = self.param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):

    rclpy.init(args=args)

    drone = DroneControl()
    drone.get_logger().info('start main ')

    while drone.state is None or drone.start_altitude == 0.0:
        drone.get_logger().info(f'{drone.robot_name} waiting for drone state')
        rclpy.spin_once(drone)


    drone.get_logger().info(f'{drone.robot_name} before declare param')

    drone.get_logger().info(f'{drone.robot_name} before arming')
    time.sleep(3)
    result = drone.arm_drone()
    while not result.success:
        result = drone.arm_drone()
        time.sleep(0.5)          
    #drone.wait_for_arming()


    drone.takeoff_drone(5.0)
    drone.wait_for_altitude()
    drone.set_mode("AUTO.LOITER")

    # add some movement in a loop
    while True:
        """
        mouse_x = drone.raspimouse_pose.transform.translation.x
        mouse_y = drone.raspimouse_pose.transform.translation.y
        drone.wait_for_position(mouse_x + 3.0, mouse_y + 3.0, 5.0)
        drone.wait_for_position(mouse_x - 3.0, mouse_y + 3.0, 5.0)
        drone.wait_for_position(mouse_x - 3.0, mouse_y - 3.0, 5.0)
        drone.wait_for_position(mouse_x + 3.0, mouse_y - 3.0, 5.0)
        """
        drone.wait_for_position(5.0, 0.0, 6.0)
        drone.wait_for_position(5.0, 5.0, 4.0)
        drone.wait_for_position(0.0, 5.0, 6.0)
        drone.wait_for_position(0.0, -5.0, 4.0)


    drone.set_mode("AUTO.LAND")

    drone.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()