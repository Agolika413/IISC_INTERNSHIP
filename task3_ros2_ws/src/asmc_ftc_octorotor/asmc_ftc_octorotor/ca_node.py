#!/usr/bin/env python3
# pyrefly: ignore-all-errors
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64MultiArray
from actuator_msgs.msg import Actuators
import numpy as np
from scipy.optimize import lsq_linear

class ControlAllocationNode(Node):
    def __init__(self):
        super().__init__('ca_node')
        
        self.sub = self.create_subscription(Wrench, '/virtual_controls', self.ca_callback, 10)
        
        # Single publisher for all motor speeds using Actuators message
        self.motor_pub = self.create_publisher(Actuators, '/motor_speed_cmd', 10)
        
        self.fault_sub = self.create_subscription(Float64MultiArray, '/fault_injection', self.fault_callback, 10)
        
        Ld = 0.23
        b_t = 2.8e-6
        d_t = 7.5e-8
        
        self.Ba = np.array([
            [1, 1, 1, 1, 1, 1, 1, 1],
            [0, 0, Ld, Ld, 0, 0, -Ld, -Ld],
            [-Ld, -Ld, 0, 0, Ld, Ld, 0, 0],
            [d_t/b_t, -d_t/b_t, -d_t/b_t, d_t/b_t, d_t/b_t, -d_t/b_t, -d_t/b_t, d_t/b_t]
        ])
        
        self.L = np.ones(8)
        self.U_MAX = 4.5
        
    def fault_callback(self, msg):
        if len(msg.data) == 8:
            self.L = np.array(msg.data)
            self.get_logger().info(f"Received fault parameters: {self.L}")
            
    def ca_callback(self, msg):
        nu_d = np.array([msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z])
        
        L_safe = np.maximum(self.L, 1e-8)
        B_eff = self.Ba @ np.diag(L_safe)
        
        # Pseudo-inverse based Control Allocation (minimizes ||u||^2)
        # This prevents the solver from outputting wildly asymmetric thrusts for symmetric targets
        pinv_B = np.linalg.pinv(B_eff)
        u = pinv_B @ nu_d
        u = np.clip(u, 0.0, self.U_MAX)
        
        # Apply fault before sending to gazebo
        u_actual = self.L * u
        
        # Convert force to angular velocity for Gazebo motor plugin
        # F = b_t * w^2 => w = sqrt(F / b_t)
        b_t = 2.8e-6
        w_cmd = np.sqrt(np.maximum(u_actual / b_t, 0.0))
        
        # Publish all 8 motor speeds in a single Actuators message
        act_msg = Actuators()
        act_msg.velocity = [float(w) for w in w_cmd]
        act_msg.normalized = [float(w / 2000.0) for w in w_cmd]
        self.motor_pub.publish(act_msg)
        
        self.get_logger().info(
            f"Wrench Z={msg.force.z:.1f} -> w0={w_cmd[0]:.0f} w1={w_cmd[1]:.0f}",
            throttle_duration_sec=2.0
        )

def main(args=None):
    rclpy.init(args=args)
    node = ControlAllocationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
