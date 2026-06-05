#!/usr/bin/env python3
# pyrefly: ignore-all-errors
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class FaultInjectorNode(Node):
    def __init__(self):
        super().__init__('fault_injector_node')
        self.pub = self.create_publisher(Float64MultiArray, '/fault_injection', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
    def timer_callback(self):
        t = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        L = [1.0] * 8
        
        # Scenario 2: Fault at t=20s
        if t >= 20.0:
            L[0] = 0.0 # Motor 1 completely dead
            L[4] = 0.6 # Motor 5 loss of 40%
            
        msg = Float64MultiArray()
        msg.data = L
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FaultInjectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
