#!/usr/bin/env python3
"""
Image converter node to convert mono16/bgra8/bgr8 to mono8 format
This allows ORB-SLAM3 to work with different camera output formats
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageConverter(Node):
    def __init__(self):
        super().__init__('image_converter')
        
        self.bridge = CvBridge()
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Publisher for mono8 converted image
        self.publisher = self.create_publisher(
            Image,
            '/camera/image_mono8',
            10)
        
        self.get_logger().info('Image converter node started - converting to mono8')
        self.frame_count = 0

    def image_callback(self, msg):
        try:
            self.frame_count += 1

            # Log every 100 frames to show activity
            if self.frame_count % 100 == 0:
                self.get_logger().info(f'Processed {self.frame_count} frames, encoding: {msg.encoding}')

            # Convert ROS image to OpenCV format based on encoding
            if msg.encoding == 'bgra8':
                # Convert bgra8 to bgr8 first, then to grayscale
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                cv_image_mono = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            elif msg.encoding == 'bgr8':
                # Convert bgr8 to grayscale
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                cv_image_mono = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            elif msg.encoding == 'rgb8':
                # Convert rgb8 to grayscale
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                cv_image_mono = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
            elif msg.encoding == 'mono16':
                # Convert mono16 to mono8 by scaling down
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
                cv_image_mono = (cv_image / 256).astype(np.uint8)
            elif msg.encoding == 'mono8':
                # Already mono8, just pass through
                cv_image_mono = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            else:
                self.get_logger().warn(f'Unsupported encoding: {msg.encoding}')
                return

            # Convert back to ROS message
            msg_out = self.bridge.cv2_to_imgmsg(cv_image_mono, encoding='mono8')
            msg_out.header = msg.header  # Preserve timestamp and frame_id

            # Publish converted image
            self.publisher.publish(msg_out)

        except Exception as e:
            self.get_logger().error(f'Error converting image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    image_converter = ImageConverter()
    
    try:
        rclpy.spin(image_converter)
    except KeyboardInterrupt:
        pass
    
    image_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
