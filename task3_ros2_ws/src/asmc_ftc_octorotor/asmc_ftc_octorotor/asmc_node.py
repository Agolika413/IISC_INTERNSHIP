#!/usr/bin/env python3
# pyrefly: ignore-all-errors
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
import numpy as np
import math

class ASMCNode(Node):
    def __init__(self):
        super().__init__('asmc_node')
        
        # Publisher for virtual controls [Uz, Up, Ut, Uy]
        self.cmd_pub = self.create_publisher(Wrench, '/virtual_controls', 10)
        
        # Subscriber for state (Odometry from Gazebo)
        self.odom_sub = self.create_subscription(Odometry, '/odometry', self.odom_callback, 10)
        
        # Timer for control loop at 200 Hz
        self.dt = 0.005
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        # State variables
        self.state = np.zeros(8) # [z, zd, ph, phd, th, thd, ps, psd]
        self.t_start = self.get_clock().now().nanoseconds / 1e9
        
        # Parameters (matching paper)
        self.m = 2.0
        self.Ixx = 0.0820
        self.Iyy = 0.0845
        self.Izz = 0.1377
        self.g = 9.81
        self.Ld = 0.23
        self.Kd = np.array([0.01, 0.012, 0.012, 0.012])
        
        self.K1 = np.array([25., 100., 100., 25.])
        self.K2 = np.array([10., 20., 20., 10.])
        self.KS = np.array([5., 10., 10., 5.])
        self.PHI = 0.2
        self.ADAPT_GAIN = np.array([0.5, 2.0, 2.0, 0.5])
        
        self.Gh = np.array([self.m, self.Ixx, self.Iyy, self.Izz])
        self.ie = np.zeros(4)
        
    def euler_from_quaternion(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw
        
    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        
        roll, pitch, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        ang_vel = msg.twist.twist.angular
        
        # Body frame approximation for simplicity 
        self.state = np.array([
            pos.z, vel.z,
            roll, ang_vel.x,
            pitch, ang_vel.y,
            yaw, ang_vel.z
        ])
        
    def get_reference(self, t):
        import scipy.signal
        Z_REF = 5.0
        P_AMP = np.deg2rad(4.0)
        freq = np.pi / 20.0
        td = P_AMP * scipy.signal.square(freq * t)
        tdd = 0.0
        return np.array([Z_REF, 0, td, 0]), np.array([0, 0, tdd, 0]), np.zeros(4)
        
    def sat(self, s):
        # softer saturation to prevent high frequency chatter
        return np.clip(s / self.PHI, -1.0, 1.0)
        
    def control_loop(self):
        t = (self.get_clock().now().nanoseconds / 1e9) - self.t_start
        s = self.state
        
        pd, vd, ad = self.get_reference(t)
        
        e = np.array([s[0]-pd[0], s[2]-pd[1], s[4]-pd[2], s[6]-pd[3]])
        ed = np.array([s[1]-vd[0], s[3]-vd[1], s[5]-vd[2], s[7]-vd[3]])
        
        for i in range(4):
            if abs(e[i]) < 0.5: self.ie[i] += e[i] * self.dt
            self.ie[i] *= 0.99  # bleed off integral to prevent permanent windup
            self.ie[i] = np.clip(self.ie[i], -0.2, 0.2)
            
        sig = ed + self.K2*e + self.K1*self.ie
        
        # f_i(x)
        phd, thd, psd = s[3], s[5], s[7]
        f = np.array([
            -self.g - (self.Kd[0]/self.m) * s[1],
            thd*psd*(self.Iyy-self.Izz)/self.Ixx - (self.Kd[1]*self.Ld/self.Ixx) * s[3],
            phd*psd*(self.Izz-self.Ixx)/self.Iyy - (self.Kd[2]*self.Ld/self.Iyy) * s[5],
            phd*thd*(self.Ixx-self.Iyy)/self.Izz - (self.Kd[3]/self.Izz) * s[7]
        ])
        
        ff = ad - self.K2*ed - self.K1*e - f
        
        # Adaptation
        sd = sig - self.PHI * self.sat(sig)
        Gd = self.ADAPT_GAIN * (-ff + self.KS * self.sat(sig)) * sd
        
        self.Gh = np.maximum(self.Gh + Gd * self.dt, 1e-4)
        self.Gh = np.clip(self.Gh, 0.1, 10.0) # Prevent gain windup
        nu = self.Gh * ff - self.Gh * self.KS * self.sat(sig)
        
        msg = Wrench()
        msg.force.z = float(np.clip(nu[0], 5.0, 35.0))
        # Strictly limit torques so they don't starve thrust for other axes
        msg.torque.x = float(np.clip(nu[1], -0.5, 0.5))
        msg.torque.y = float(np.clip(nu[2], -0.5, 0.5))
        msg.torque.z = float(np.clip(nu[3], -0.05, 0.05))
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ASMCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
