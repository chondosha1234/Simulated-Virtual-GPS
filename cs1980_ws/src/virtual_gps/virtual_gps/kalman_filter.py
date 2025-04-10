import rclpy
from rclpy.node import Node
import numpy as np 
from numpy import *
from numpy.linalg import inv, det
import math
from tf_transformations import euler_from_quaternion

from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class KalmanFilterNode(Node):

    def __init__(self):
        super().__init__('kalman_filter')

        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value

        # time step of mobile movement 
        self.dt = 0.1

        # initialization of state matrices 
        # X = [x, y, vx, vy]  -- starts as stationary -- only going to measure x/y because z remains unchanged 
        self.X = array([[0.0], [0.0], [0.0], [0.0]])

        self.P = diag((0.01, 0.01, 0.01, 0.01))

        self.A = array([[1, 0, self.dt , 0], [0, 1, 0, self.dt], [0, 0, 1, 0], [0, 0, 0, 1]])

        self.Q = eye(self.X.shape[0])

        self.B = eye(self.X.shape[0])

        self.U = zeros((self.X.shape[0], 1))

        # Measurement matrices
        self.Y = array([
            [self.X[0,0] + abs(random.randn(1)[0])], [self.X[1,0] + abs(random.randn(1)[0])], [self.X[2,0]], [self.X[3,0]]
        ])

        #self.H = array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.H = np.eye(4)

        self.R = eye(self.Y.shape[0])

        self.z_coord = 0.013

        self.gps_sub = self.create_subscription(
            TransformStamped, 
            f'/{self.robot_name}/gps', 
            self.gps_callback, 
            10
        )
    
        self.odom_sub = self.create_subscription(
            Odometry, 
            f'/odom', 
            self.odom_callback, 
            10
        )
        

        self.filtered_gps = self.create_publisher(TransformStamped, f'/{self.robot_name}/filtered_gps', 10)

        self.prediction_timer = self.create_timer(0.1, self.prediction)

        self.update_timer = self.create_timer(0.5, self.update)
    

    def gps_callback(self, msg):
        #self.Y = np.array([
         #   [msg.transform.translation.x],
          #  [msg.transform.translation.y]
        #])
        #self.z_coord = msg.transform.translation.z
        self.Y[0,0] = msg.transform.translation.x
        self.Y[1,0] = msg.transform.translation.y

    def odom_callback(self, msg):
        
        # extract the linear velocity and angular velocity from robot odom
        vx = msg.twist.twist.linear.x 

        yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])[2]

        # convert the robot x speed to world vx,vy frame 
        vx_world = vx * cos(yaw)
        vy_world = vx * sin(yaw)

        self.Y[2,0] = vx_world 
        self.Y[3,0] = vy_world

    """
    X : The mean state estimate of the previous step ( k-1 ).
    P : The state covariance of previous step (k-1).
    A : The transition n x n matrix.
    Q : The process noise covariance matrix.
    B : The input effect matrix.
    U : The control input.
    """
    #def prediction(self, X, P, A, Q, B, U):
    def prediction(self):
        self.X = dot(self.A, self.X) + dot(self.B, self.U)
        self.P = dot(self.A, dot(self.P, self.A.T)) + self.Q
        
        #self.get_logger().info(f'covariance matrix: \n{self.P}')

        # publish next predicted position from X matrix 
        msg = TransformStamped()
        msg.transform.translation.x = self.X[0,0]
        msg.transform.translation.y = self.X[1,0]
        msg.transform.translation.z = self.z_coord 
        self.filtered_gps.publish(msg)

    """
    K : the Kalman Gain matrix
    IM : the Mean of predictive distribution of Y
    IS : the Covariance or predictive mean of Y
    LH : the Predictive probability (likelihood) of measurement which is
         computed using the function gauss_pdf.
    """
    #def update(self, X, P, Y, H, R):
    def update(self):

        IM = dot(self.H, self.X)
        IS = self.R + dot(self.H, dot(self.P, self.H.T))
        K = dot(self.P, dot(self.H.T, inv(IS)))
        self.X = self.X + dot(K, (self.Y - IM))
        self.P = self.P - dot(K, dot(IS, K.T))
        LH = self.gauss_pdf(self.Y, IM, IS)
        self.get_logger().info(f'Likelyhood that this measurement is accurate: {LH}')
        #return (X, P, K, IM, IS, LH)

        # publish next predicted position from X matrix 
        msg = TransformStamped()
        msg.transform.translation.x = self.X[0,0]
        msg.transform.translation.y = self.X[1,0]
        msg.transform.translation.z = self.z_coord 
        self.filtered_gps.publish(msg)


    def gauss_pdf(self, X, M, S):
        if M.shape[1] == 1:
            DX = X - tile(M, X.shape[1])
            E = 0.5 * np.sum(DX * (dot(inv(S), DX)), axis=0)
            E = E + 0.5 * M.shape[0] * math.log(2 * math.pi) + 0.5 * math.log(det(S))
            P = np.exp(-E)
        elif X.shape[1] == 1:
            DX = tile(X, M.shape[1]) - M
            E = 0.5 * np.sum(DX * (dot(inv(S), DX)), axis=0) 
            E = E + 0.5 * M.shape[0] * math.log(2 * math.pi) + 0.5 * math.log(det(S))   # np.det?
            P = np.exp(-E)
        else:
            DX = X - M 
            E = 0.5 * np.sum(DX * dot(inv(S), DX))
            E = E + 0.5 * M.shape[0] * math.log(2 * math.pi) + 0.5 * math.log(det(S))   # np.det?
            P = np.exp(-E)

        return (P[0], E[0])


def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()