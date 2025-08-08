#!/usr/bin/env python3
"""
Image converter node to convert mono16 to mono8 format
This allows ORB-SLAM3 to work with mono16 camera output
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
        
        # Subscribe to mono16 camera topic
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
        
        self.get_logger().info('Image converter node started - converting mono16 to mono8')

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            if msg.encoding == 'mono16':
                # Convert mono16 to cv2 image
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
                
                # Convert 16-bit to 8-bit by scaling
                # Method 1: Simple scaling (dividing by 256)
                cv_image_mono8 = (cv_image / 256).astype(np.uint8)
                
                # Method 2: More sophisticated approach using dynamic range
                # cv_image_mono8 = cv2.convertScaleAbs(cv_image, alpha=255.0/cv_image.max())
                
                # Convert back to ROS message
                msg_out = self.bridge.cv2_to_imgmsg(cv_image_mono8, encoding='mono8')
                msg_out.header = msg.header  # Preserve timestamp and frame_id
                
                # Publish converted image
                self.publisher.publish(msg_out)
                
            elif msg.encoding == 'mono8':
                # Already mono8, just republish
                self.publisher.publish(msg)
                
            else:
                self.get_logger().warn(f'Unsupported encoding: {msg.encoding}')
                
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
