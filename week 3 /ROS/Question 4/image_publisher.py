#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

VIDEO_SOURCE = 0 

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('webcam_image_publisher')
        
        self.declare_parameter('video_source', VIDEO_SOURCE)
        source = self.get_parameter('video_source').get_parameter_value().integer_value
        
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        timer_period = 0.033
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f"Attempting to open video source: {source}...")
        self.cap = cv2.VideoCapture(source)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video source: {source}")
        else:
            self.get_logger().info(f"Video source opened. Publishing to /camera/image_raw")
            
        self.bridge = CvBridge()

    def timer_callback(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        
        if ret:
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image_message.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(image_message)
        else:
            self.get_logger().warn("Failed to retrieve frame from video source.")

    def on_shutdown(self):
        self.get_logger().info("Shutting down...")
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    
    image_publisher = ImagePublisher()
    
    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher.on_shutdown()
        image_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
