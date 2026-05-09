#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # Task 1.b: Subscribe to the Turtlebot's camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.image_callback,
            10)
            
        # Publisher for the error
        self.error_pub = self.create_publisher(Float64, '/error', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info("Vision Node started. Looking for green spheres...")

    def image_callback(self, msg):
        # 1. Converting ROS Image to OpenCV Image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = cv_image.shape
        
        # 2. BGR to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 3. The Green Color range in HSV
        # For checking the basis of values, check the README File
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        
        # Creating a mask to isolate the green color
        mask = cv2.inRange(hsv_image, lower_green, upper_green)
        
        # Simple morphology to clean up noise 
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # 4. Find Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        error_msg = Float64()
        
        if len(contours) > 0:
            # Get the largest contour (assuming it's sphere)
            largest_contour = max(contours, key=cv2.contourArea)
            
            # 5. Using cv2.moments to find the center
            M = cv2.moments(largest_contour)
            if M['m00'] > 0: # Preventing division by zero
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                # Draw a circle at the center for visual debugging
                cv2.circle(cv_image, (cx, cy), 10, (0, 0, 255), -1)
                
                # 6. Calculate Horizontal Error
                image_center_x = width / 2.0
                # Error is positive if object is to the left, negative if to the right
                error_value = image_center_x - cx 
                
                error_msg.data = float(error_value)
                self.error_pub.publish(error_msg)
                
        else:
            # OUT OF FRAME RECOVERY
            error_msg.data = 9999.0
            self.error_pub.publish(error_msg)
            self.get_logger().info("Object out of frame!")

        
        # Automatically opens the video feed window when the node is launched. This is for debugging purposes to visualize the camera feed and the detected object.
        cv2.imshow("Camera Feed", cv_image)
        
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    
    vision_node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()