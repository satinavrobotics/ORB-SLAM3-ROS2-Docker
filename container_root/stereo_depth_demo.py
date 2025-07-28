#!/usr/bin/env python3

import numpy as np
import cv2 as cv
import json
import threading
import time

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class StereoDepthEstimator(Node):
    def __init__(self, calibration_file="stereo_calibration.json"):
        """
        Initialize stereo depth estimator
        
        Args:
            calibration_file: Path to stereo calibration file
        """
        super().__init__('stereo_depth_estimator')
        
        # Load calibration data
        self.load_calibration(calibration_file)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Current frames
        self.current_frame_left = None
        self.current_frame_right = None
        self.frame_lock = threading.Lock()
        
        # Stereo matcher
        self.create_stereo_matcher()
        
        # Create image subscribers
        self.image_subscription_left = self.create_subscription(
            Image,
            '/camera_left/image_raw',
            self.image_callback_left,
            10
        )
        
        self.image_subscription_right = self.create_subscription(
            Image,
            '/camera_right/image_raw',
            self.image_callback_right,
            10
        )
        
        print("Stereo Depth Estimator initialized")
        print("Press 'q' to quit, 's' to save depth map, 'c' to change stereo parameters")
    
    def load_calibration(self, filename):
        """Load stereo calibration from file"""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            # Load calibration matrices
            self.camera_matrix_left = np.array(data["camera_matrix_left"])
            self.dist_coeffs_left = np.array(data["distortion_coefficients_left"])
            self.camera_matrix_right = np.array(data["camera_matrix_right"])
            self.dist_coeffs_right = np.array(data["distortion_coefficients_right"])
            
            # Load stereo parameters
            self.R1 = np.array(data["rectification_left"])
            self.R2 = np.array(data["rectification_right"])
            self.P1 = np.array(data["projection_left"])
            self.P2 = np.array(data["projection_right"])
            self.Q = np.array(data["disparity_to_depth_matrix"])
            
            self.baseline = data["baseline_distance_mm"]
            
            print(f"✓ Loaded stereo calibration from {filename}")
            print(f"Baseline: {self.baseline:.2f}mm")
            
            return True
        except Exception as e:
            print(f"✗ Error loading calibration: {e}")
            return False
    
    def create_stereo_matcher(self):
        """Create stereo block matcher"""
        # StereoBM parameters
        self.stereo_bm = cv.StereoBM_create(numDisparities=96, blockSize=15)
        
        # StereoSGBM parameters (better quality, slower)
        self.stereo_sgbm = cv.StereoSGBM_create(
            minDisparity=0,
            numDisparities=96,
            blockSize=11,
            P1=8 * 3 * 11**2,
            P2=32 * 3 * 11**2,
            disp12MaxDiff=1,
            uniquenessRatio=15,
            speckleWindowSize=0,
            speckleRange=2,
            preFilterCap=63,
            mode=cv.STEREO_SGBM_MODE_SGBM_3WAY
        )
        
        self.current_matcher = self.stereo_sgbm
        self.matcher_name = "SGBM"
        
        print("Using StereoSGBM matcher (high quality)")
    
    def image_callback_left(self, msg):
        """Callback for left camera"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            with self.frame_lock:
                self.current_frame_left = cv_image.copy()
        except Exception as e:
            self.get_logger().error(f'Error converting left image: {e}')
    
    def image_callback_right(self, msg):
        """Callback for right camera"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            with self.frame_lock:
                self.current_frame_right = cv_image.copy()
        except Exception as e:
            self.get_logger().error(f'Error converting right image: {e}')
    
    def compute_depth_map(self, img_left, img_right):
        """Compute depth map from stereo pair"""
        # Get image dimensions
        h, w = img_left.shape[:2]
        
        # Create rectification maps if not already created
        if not hasattr(self, 'map1_left'):
            self.map1_left, self.map2_left = cv.initUndistortRectifyMap(
                self.camera_matrix_left, self.dist_coeffs_left, self.R1, self.P1, (w, h), cv.CV_16SC2
            )
            self.map1_right, self.map2_right = cv.initUndistortRectifyMap(
                self.camera_matrix_right, self.dist_coeffs_right, self.R2, self.P2, (w, h), cv.CV_16SC2
            )
        
        # Rectify images
        img_left_rect = cv.remap(img_left, self.map1_left, self.map2_left, cv.INTER_LINEAR)
        img_right_rect = cv.remap(img_right, self.map1_right, self.map2_right, cv.INTER_LINEAR)
        
        # Compute disparity
        disparity = self.current_matcher.compute(img_left_rect, img_right_rect).astype(np.float32) / 16.0
        
        # Convert disparity to depth
        depth_map = cv.reprojectImageTo3D(disparity, self.Q)
        
        # Extract Z component (depth)
        depth = depth_map[:, :, 2]
        
        # Filter invalid depths
        depth[depth <= 0] = 0
        depth[depth > 10000] = 0  # Remove points beyond 10 meters
        
        return disparity, depth, img_left_rect, img_right_rect
    
    def create_depth_colormap(self, depth, max_depth=2000):
        """Create colored depth map for visualization"""
        # Normalize depth to 0-255
        depth_norm = np.clip(depth / max_depth * 255, 0, 255).astype(np.uint8)
        
        # Apply colormap
        depth_colored = cv.applyColorMap(depth_norm, cv.COLORMAP_JET)
        
        # Set invalid points to black
        mask = depth <= 0
        depth_colored[mask] = [0, 0, 0]
        
        return depth_colored
    
    def run_depth_estimation(self):
        """Main loop for depth estimation"""
        cv.namedWindow('Stereo Rectified', cv.WINDOW_AUTOSIZE)
        cv.namedWindow('Disparity Map', cv.WINDOW_AUTOSIZE)
        cv.namedWindow('Depth Map', cv.WINDOW_AUTOSIZE)
        
        save_counter = 0
        
        print("Depth estimation running...")
        print("Controls:")
        print("  'q' - Quit")
        print("  's' - Save current depth map")
        print("  'b' - Switch to Block Matching (fast)")
        print("  'g' - Switch to SGBM (high quality)")
        print("  '+' - Increase max depth range")
        print("  '-' - Decrease max depth range")
        
        max_depth_range = 2000  # mm
        
        while True:
            rclpy.spin_once(self, timeout_sec=0.001)
            
            with self.frame_lock:
                if self.current_frame_left is None or self.current_frame_right is None:
                    time.sleep(0.01)
                    continue
                
                img_left = self.current_frame_left.copy()
                img_right = self.current_frame_right.copy()
            
            try:
                # Compute depth
                disparity, depth, left_rect, right_rect = self.compute_depth_map(img_left, img_right)
                
                # Create visualizations
                left_rect_color = cv.cvtColor(left_rect, cv.COLOR_GRAY2BGR)
                right_rect_color = cv.cvtColor(right_rect, cv.COLOR_GRAY2BGR)
                
                # Show rectified images side by side with epipolar lines
                rectified_combined = np.hstack((left_rect_color, right_rect_color))
                h, w = rectified_combined.shape[:2]
                for i in range(0, h, 50):
                    cv.line(rectified_combined, (0, i), (w, i), (0, 255, 0), 1)
                
                # Create disparity visualization
                disparity_norm = cv.normalize(disparity, None, 0, 255, cv.NORM_MINMAX, cv.CV_8U)
                disparity_colored = cv.applyColorMap(disparity_norm, cv.COLORMAP_JET)
                
                # Create depth visualization
                depth_colored = self.create_depth_colormap(depth, max_depth_range)
                
                # Add information overlays
                cv.putText(rectified_combined, f"Baseline: {self.baseline:.1f}mm", (10, 30), 
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv.putText(rectified_combined, f"Matcher: {self.matcher_name}", (10, 60), 
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                cv.putText(disparity_colored, f"Disparity Range: 0-{self.current_matcher.getNumDisparities()}", 
                          (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                cv.putText(depth_colored, f"Depth Range: 0-{max_depth_range}mm", (10, 30), 
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                # Calculate some statistics
                valid_depth = depth[depth > 0]
                if len(valid_depth) > 0:
                    min_depth = np.min(valid_depth)
                    max_depth = np.max(valid_depth)
                    mean_depth = np.mean(valid_depth)
                    
                    cv.putText(depth_colored, f"Min: {min_depth:.0f}mm", (10, 60), 
                              cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv.putText(depth_colored, f"Max: {max_depth:.0f}mm", (10, 80), 
                              cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv.putText(depth_colored, f"Mean: {mean_depth:.0f}mm", (10, 100), 
                              cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Display images
                cv.imshow('Stereo Rectified', rectified_combined)
                cv.imshow('Disparity Map', disparity_colored)
                cv.imshow('Depth Map', depth_colored)
                
            except Exception as e:
                print(f"Error in depth computation: {e}")
                time.sleep(0.1)
                continue
            
            # Handle keyboard input
            key = cv.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                # Save depth map
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                cv.imwrite(f"depth_map_{timestamp}.png", depth_colored)
                cv.imwrite(f"disparity_{timestamp}.png", disparity_colored)
                np.save(f"depth_data_{timestamp}.npy", depth)
                save_counter += 1
                print(f"Saved depth map #{save_counter}")
            elif key == ord('b'):
                # Switch to Block Matching
                self.current_matcher = self.stereo_bm
                self.matcher_name = "BM"
                print("Switched to Block Matching (fast)")
            elif key == ord('g'):
                # Switch to SGBM
                self.current_matcher = self.stereo_sgbm
                self.matcher_name = "SGBM"
                print("Switched to SGBM (high quality)")
            elif key == ord('+') or key == ord('='):
                max_depth_range += 500
                print(f"Max depth range: {max_depth_range}mm")
            elif key == ord('-'):
                max_depth_range = max(500, max_depth_range - 500)
                print(f"Max depth range: {max_depth_range}mm")
        
        cv.destroyAllWindows()

def main():
    """Main function"""
    print("=== Stereo Depth Estimation Demo ===")
    
    # Check for calibration file
    calibration_file = input("Enter calibration file path (default: stereo_calibration.json): ").strip()
    if not calibration_file:
        calibration_file = "stereo_calibration.json"
    
    print(f"Loading calibration from: {calibration_file}")
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create depth estimator
        depth_estimator = StereoDepthEstimator(calibration_file)
        
        # Run depth estimation
        depth_estimator.run_depth_estimation()
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()
        cv.destroyAllWindows()

if __name__ == "__main__":
    main()