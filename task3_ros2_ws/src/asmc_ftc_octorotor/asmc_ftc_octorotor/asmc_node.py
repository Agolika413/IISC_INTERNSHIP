#!/usr/bin/env python3
# pyrefly: ignore-all-errors
"""
ASMC Controller Node for Octorotor FTC.

Strategy: Use a simple PD controller to take off and stabilize first,
then switch to full ASMC once airborne and stable. This prevents
the adaptive gains from exploding while the drone is on the ground.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
import numpy as np
import math


class ASMCNode(Node):
    def __init__(self):
        super().__init__('asmc_node')

        self.cmd_pub = self.create_publisher(Wrench, '/virtual_controls', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry', self.odom_callback, 10)

        self.dt = 0.005
        self.timer = self.create_timer(self.dt, self.control_loop)

        # State
        self.state = np.zeros(8)
        self.odom_received = False
        self.t_start = None

        # Physical parameters
        self.m   = 2.0
        self.g   = 9.81
        self.Ixx = 0.0820
        self.Iyy = 0.0845
        self.Izz = 0.1377

        # ASMC gains
        self.K1 = np.array([25.0, 100.0, 100.0, 25.0])
        self.K2 = np.array([10.0,  20.0,  20.0, 10.0])
        self.KS = np.array([ 5.0,  10.0,  10.0,  5.0])
        self.PHI = 0.2
        self.ADAPT_GAIN = np.array([0.5, 2.0, 2.0, 0.5])

        # Adaptive gains initialized to nominal
        self.Gh = np.array([self.m, self.Ixx, self.Iyy, self.Izz])
        self.ie = np.zeros(4)

    @staticmethod
    def euler_from_quaternion(q):
        x, y, z, w = q.x, q.y, q.z, q.w
        roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        sinp  = max(-1.0, min(1.0, 2*(w*y - z*x)))
        pitch = math.asin(sinp)
        yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return roll, pitch, yaw

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        roll, pitch, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        ang = msg.twist.twist.angular
        self.state = np.array([
            pos.z, vel.z, roll, ang.x, pitch, ang.y, yaw, ang.z])
        if not self.odom_received:
            self.odom_received = True
            self.t_start = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(f"Odom OK. z={pos.z:.2f}")

    def sat(self, s):
        return np.clip(s / self.PHI, -1.0, 1.0)

    def get_reference(self, t):
        Z_REF = 5.0
        P_AMP = np.deg2rad(4.0)
        freq  = np.pi / 20.0
        try:
            import scipy.signal
            td = P_AMP * scipy.signal.square(freq * t)
        except ImportError:
            td = P_AMP * np.sign(np.sin(freq * t))
        return np.array([Z_REF, 0.0, td, 0.0])

    def control_loop(self):
        if not self.odom_received:
            return

        t = (self.get_clock().now().nanoseconds / 1e9) - self.t_start
        s = self.state
        z, zd, phi, phd, theta, thd, psi, psd = s

        pd = self.get_reference(t)
        Z_REF = pd[0]

        # ── PHASE 1: Simple PD takeoff (t < 5s) ─────────────────────
        # This gets the drone into the air safely before ASMC engages.
        if t < 5.0:
            # Altitude: PD + gravity feedforward
            ez  = z - Z_REF
            ezd = zd
            Uz  = self.m * self.g - 8.0 * ez - 4.0 * ezd
            # Attitude: PD to keep level
            Uphi   = -20.0 * phi   - 5.0 * phd
            Utheta = -20.0 * theta - 5.0 * thd
            Upsi   = -5.0  * psi   - 2.0 * psd

            msg = Wrench()
            msg.force.z  = float(np.clip(Uz, 0.0, 35.0))
            msg.torque.x = float(np.clip(Uphi,   -1.0, 1.0))
            msg.torque.y = float(np.clip(Utheta,  -1.0, 1.0))
            msg.torque.z = float(np.clip(Upsi,   -0.2, 0.2))
            self.cmd_pub.publish(msg)

            if int(t * 10) % 20 == 0:
                self.get_logger().info(
                    f"[PD] t={t:.1f} z={z:.2f} Uz={Uz:.1f} "
                    f"phi={math.degrees(phi):.1f} theta={math.degrees(theta):.1f}",
                    throttle_duration_sec=2.0)
            return

        # ── PHASE 2: Full ASMC (t >= 5s) ────────────────────────────
        e  = np.array([z - pd[0], phi - pd[1], theta - pd[2], psi - pd[3]])
        ed = np.array([zd, phd, thd, psd])

        # Integral with anti-windup
        for i in range(4):
            if abs(e[i]) < 0.5:
                self.ie[i] += e[i] * self.dt
            self.ie[i] = np.clip(self.ie[i], -2.0, 2.0)

        sig = ed + self.K2 * e + self.K1 * self.ie

        # f_i(x): known autonomous dynamics (NO drag — matches paper)
        f = np.array([
            -self.g,
            thd * psd * (self.Iyy - self.Izz) / self.Ixx,
            phd * psd * (self.Izz - self.Ixx) / self.Iyy,
            phd * thd * (self.Ixx - self.Iyy) / self.Izz
        ])

        ff = -self.K2 * ed - self.K1 * e - f

        # Adaptation law
        sd = sig - self.PHI * self.sat(sig)
        Gd = self.ADAPT_GAIN * (-ff + self.KS * self.sat(sig)) * sd

        self.Gh = np.maximum(self.Gh + Gd * self.dt, 1e-4)
        self.Gh = np.clip(self.Gh, 0.1, 10.0)

        nu = self.Gh * ff - self.Gh * self.KS * self.sat(sig)

        msg = Wrench()
        msg.force.z  = float(np.clip(nu[0], 0.0, 35.0))
        msg.torque.x = float(np.clip(nu[1], -2.0, 2.0))
        msg.torque.y = float(np.clip(nu[2], -2.0, 2.0))
        msg.torque.z = float(np.clip(nu[3], -0.5, 0.5))
        self.cmd_pub.publish(msg)

        if int(t * 10) % 20 == 0:
            self.get_logger().info(
                f"[ASMC] t={t:.1f} z={z:.2f} Uz={nu[0]:.1f} "
                f"phi={math.degrees(phi):.1f} theta={math.degrees(theta):.1f} "
                f"Gh={self.Gh[0]:.2f}",
                throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ASMCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
