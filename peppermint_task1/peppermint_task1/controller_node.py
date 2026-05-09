#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped  
from sensor_msgs.msg import LaserScan
import math

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        self.error_sub = self.create_subscription(
            Float64, '/error', self.error_callback, 10)
            
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
            
        
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        self.current_error = 9999.0  
        self.front_distance = float('inf') 
        
        self.Kp = 0.002        
        self.error_tolerance = 50.0  
        self.stop_distance = 0.35    
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Controller Node started. Waiting for vision data...")

    def error_callback(self, msg):
        self.current_error = msg.data

    def scan_callback(self, msg):
        front_ranges = msg.ranges[0:10] + msg.ranges[-10:]
        valid_ranges = [r for r in front_ranges if 0.0 < r < 10.0]
        
        if valid_ranges:
            self.front_distance = min(valid_ranges)
        else:
            self.front_distance = float('inf')

    def control_loop(self):
        
        cmd_msg = TwistStamped()
        
        # Add the required timestamp header
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = "base_link"
        
        if self.front_distance <= self.stop_distance:
            self.get_logger().info(f"Reached object! Distance: {self.front_distance:.2f}m. Stopping.", once=True)
            cmd_msg.twist.linear.x = 0.0
            cmd_msg.twist.angular.z = 0.0
            self.cmd_pub.publish(cmd_msg)
            return 

        if self.current_error == 9999.0:
            cmd_msg.twist.linear.x = 0.0
            cmd_msg.twist.angular.z = 0.3 
            
        else:
            angular_velocity = self.Kp * self.current_error
            angular_velocity = max(min(angular_velocity, 0.5), -0.5)
            
            # CHANGED: Assign velocities inside the 'twist' sub-object
            cmd_msg.twist.angular.z = angular_velocity
            
            if abs(self.current_error) < self.error_tolerance:
                cmd_msg.twist.linear.x = 0.15 
            else:
                cmd_msg.twist.linear.x = 0.0
                
        self.cmd_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    
    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = TwistStamped()
        controller_node.cmd_pub.publish(stop_msg)
        controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  