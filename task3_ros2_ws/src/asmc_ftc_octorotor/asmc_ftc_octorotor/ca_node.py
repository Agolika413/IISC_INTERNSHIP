#!/usr/bin/env python3
# pyrefly: ignore-all-errors
"""
Control Allocation Node for Octorotor FTC.

Takes the virtual control input [Uz, U_phi, U_theta, U_psi] from the ASMC
and distributes it to 8 individual motor thrusts using a pseudo-inverse.

The Gazebo MulticopterMotorModel plugin expects angular velocity (rad/s),
so we convert: F_i = b_t * w_i^2  =>  w_i = sqrt(F_i / b_t).

IMPORTANT: The Gazebo motor plugin itself does NOT know about faults.
The fault_injector publishes L = [l1..l8] which we use to modify the
allocation matrix. The allocated thrusts are the COMMANDED thrusts;
Gazebo applies them directly. We do NOT multiply by L again.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64MultiArray
from actuator_msgs.msg import Actuators
import numpy as np


class ControlAllocationNode(Node):
    def __init__(self):
        super().__init__('ca_node')

        self.sub = self.create_subscription(
            Wrench, '/virtual_controls', self.ca_callback, 10)
        self.motor_pub = self.create_publisher(
            Actuators, '/motor_speed_cmd', 10)
        self.fault_sub = self.create_subscription(
            Float64MultiArray, '/fault_injection', self.fault_callback, 10)

        # ── Motor layout (coaxial, 4 arms, upper/lower on each) ──
        # Arm positions:
        #   Arm 1 (Front, +x):  rotors 0 (upper CW), 1 (lower CCW)
        #   Arm 2 (Right, +y):  rotors 2 (upper CCW), 3 (lower CW)
        #   Arm 3 (Rear,  -x):  rotors 4 (upper CW),  5 (lower CCW)
        #   Arm 4 (Left,  -y):  rotors 6 (upper CCW), 7 (lower CW)
        Ld  = 0.23
        b_t = 2.8e-6
        d_t = 7.5e-8
        kappa = d_t / b_t  # torque/thrust ratio ≈ 0.0268

        # Ba maps [u1..u8] -> [Uz, U_phi, U_theta, U_psi]
        # Row 0 (Thrust):   all motors contribute +1
        # Row 1 (Roll/phi): right arm (+Ld), left arm (-Ld)
        # Row 2 (Pitch/theta): rear arm (+Ld), front arm (-Ld)
        # Row 3 (Yaw/psi):  CW motors +kappa, CCW motors -kappa
        self.Ba = np.array([
            [ 1,      1,      1,      1,      1,      1,      1,      1     ],
            [ 0,      0,      Ld,     Ld,     0,      0,     -Ld,   -Ld    ],
            [-Ld,    -Ld,     0,      0,      Ld,     Ld,     0,      0     ],
            [ kappa, -kappa, -kappa,  kappa,  kappa, -kappa, -kappa,  kappa ]
        ])

        self.L = np.ones(8)
        self.b_t = b_t
        self.U_MAX = 4.5  # max thrust per motor (N)

    def fault_callback(self, msg):
        if len(msg.data) == 8:
            self.L = np.array(msg.data)
            self.get_logger().info(
                f"Fault: {self.L}", throttle_duration_sec=5.0)

    def ca_callback(self, msg):
        nu_d = np.array([
            msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z])

        # Build fault-aware allocation matrix
        L_safe = np.maximum(self.L, 1e-8)
        B_eff  = self.Ba @ np.diag(L_safe)

        # Weighted pseudo-inverse (penalize faulty motors)
        W  = np.diag(L_safe ** 2)
        M  = B_eff @ W @ B_eff.T + 1e-8 * np.eye(4)
        try:
            Mi = np.linalg.inv(M)
        except np.linalg.LinAlgError:
            Mi = np.linalg.pinv(M)
        u = W @ B_eff.T @ Mi @ nu_d
        u = np.clip(u, 0.0, self.U_MAX)

        # Convert thrust (N) -> angular velocity (rad/s)
        # F = b_t * w^2  =>  w = sqrt(F / b_t)
        w_cmd = np.sqrt(np.maximum(u / self.b_t, 0.0))

        # Publish
        act_msg = Actuators()
        act_msg.velocity = [float(w) for w in w_cmd]
        act_msg.normalized = [float(w / 2000.0) for w in w_cmd]
        self.motor_pub.publish(act_msg)

        self.get_logger().info(
            f"Uz={nu_d[0]:.1f} -> w=[{w_cmd[0]:.0f} {w_cmd[1]:.0f} "
            f"{w_cmd[2]:.0f} {w_cmd[3]:.0f} {w_cmd[4]:.0f} {w_cmd[5]:.0f} "
            f"{w_cmd[6]:.0f} {w_cmd[7]:.0f}]",
            throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ControlAllocationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
