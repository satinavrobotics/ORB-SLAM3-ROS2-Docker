#!/usr/bin/env python3

import numpy as np
import cv2 as cv
import os
import json
from datetime import datetime
import threading
import time

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ROS2StereoCameraCalibrator(Node):
    def __init__(self, chessboard_size=(9, 7), square_size=25.0):
        """
        Initialize the ROS2 stereo camera calibrator
        
        Args:
            chessboard_size: Tuple of (width, height) of internal corners
            square_size: Size of chessboard square in mm (for real-world scaling)
        """
        super().__init__('stereo_camera_calibrator')
        
        self.chessboard_size = chessboard_size
        self.square_size = square_size
        
        # CV Bridge for converting ROS images to OpenCV
        self.bridge = CvBridge()
        
        # Termination criteria for corner refinement
        self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        # Prepare object points (3D points in real world space)
        self.objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
        self.objp *= square_size  # Scale to real world units
        
        # Arrays to store object points and image points for both cameras
        self.objpoints = []  # 3D points in real world space
        self.imgpoints_left = []   # 2D points in left camera
        self.imgpoints_right = []  # 2D points in right camera
        
        # Individual camera calibration results
        self.camera_matrix_left = None
        self.dist_coeffs_left = None
        self.camera_matrix_right = None
        self.dist_coeffs_right = None
        
        # Stereo calibration results
        self.R = None  # Rotation matrix between cameras
        self.T = None  # Translation vector between cameras
        self.E = None  # Essential matrix
        self.F = None  # Fundamental matrix
        self.stereo_error = None
        
        # Stereo rectification results
        self.R1 = None  # Rectification transform for left camera
        self.R2 = None  # Rectification transform for right camera
        self.P1 = None  # Projection matrix for left camera
        self.P2 = None  # Projection matrix for right camera
        self.Q = None   # Disparity-to-depth mapping matrix
        self.roi_left = None   # Region of interest for left camera
        self.roi_right = None  # Region of interest for right camera
        self.map1_left = None   # Rectification maps
        self.map2_left = None
        self.map1_right = None
        self.map2_right = None
        
        # Image capture variables
        self.current_frame_left = None
        self.current_frame_right = None
        self.frame_lock = threading.Lock()
        self.capturing = False
        self.good_stereo_pairs = 0
        self.min_pairs = 15
        self.last_capture_time = 0
        self.capture_interval = 2.0
        
        # Debug variables
        self.debug_mode = True
        self.show_preprocessing = False
        
        # Create image subscribers for both cameras
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
        
        print("ROS2 Stereo Camera Calibrator initialized")
        print("Waiting for images on topics:")
        print("  Left camera:  /camera_left/image_raw")
        print("  Right camera: /camera_right/image_raw")
    
    def image_callback_left(self, msg):
        """Callback for receiving left camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            with self.frame_lock:
                self.current_frame_left = cv_image.copy()
        except Exception as e:
            self.get_logger().error(f'Error converting left image: {e}')
    
    def image_callback_right(self, msg):
        """Callback for receiving right camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            with self.frame_lock:
                self.current_frame_right = cv_image.copy()
        except Exception as e:
            self.get_logger().error(f'Error converting right image: {e}')
    
    def preprocess_image(self, image):
        """Enhanced image preprocessing for better chessboard detection"""
        processed = image.copy()
        processed = cv.GaussianBlur(processed, (5, 5), 0)
        clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        processed = clahe.apply(processed)
        processed = cv.bilateralFilter(processed, 9, 75, 75)
        return processed
    
    def detect_chessboard_robust(self, image):
        """More robust chessboard detection with multiple methods"""
        gray = image.copy()
        
        # Method 1: Standard detection
        ret1, corners1 = cv.findChessboardCorners(gray, self.chessboard_size, None)
        
        # Method 2: With preprocessing
        processed = self.preprocess_image(gray)
        ret2, corners2 = cv.findChessboardCorners(processed, self.chessboard_size, None)
        
        # Method 3: With different flags
        flags = cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE + cv.CALIB_CB_FILTER_QUADS
        ret3, corners3 = cv.findChessboardCorners(gray, self.chessboard_size, flags)
        
        # Choose the best result
        results = [(ret1, corners1, "Standard"), 
                  (ret2, corners2, "Preprocessed"), 
                  (ret3, corners3, "Adaptive")]
        
        for ret, corners, method in results:
            if ret:
                if self.debug_mode:
                    print(f"Chessboard detected using: {method}")
                return ret, corners, method
        
        return False, None, "None"
    
    def wait_for_cameras(self, timeout=10.0):
        """Wait for both camera images to start arriving"""
        print("Waiting for stereo camera images...")
        start_time = time.time()
        
        left_ready = False
        right_ready = False
        
        while time.time() - start_time < timeout:
            with self.frame_lock:
                if self.current_frame_left is not None and not left_ready:
                    h, w = self.current_frame_left.shape[:2]
                    print(f"✓ Left camera detected! Image size: {w}x{h}")
                    left_ready = True
                
                if self.current_frame_right is not None and not right_ready:
                    h, w = self.current_frame_right.shape[:2]
                    print(f"✓ Right camera detected! Image size: {w}x{h}")
                    right_ready = True
                
                if left_ready and right_ready:
                    return True
            
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        if not left_ready:
            print("✗ Left camera not detected on /camera_left/image_raw")
        if not right_ready:
            print("✗ Right camera not detected on /camera_right/image_raw")
        
        print("Make sure both camera nodes are running!")
        return False
    
    def capture_stereo_calibration_images(self, min_pairs=15, save_images=True):
        """
        Capture synchronized stereo calibration images
        """
        self.min_pairs = min_pairs
        self.good_stereo_pairs = 0
        self.capturing = True
        
        # Wait for both cameras to be available
        if not self.wait_for_cameras():
            return False
        
        # Create directory for saving images
        if save_images:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_dir = f"stereo_calibration_images_{timestamp}"
            os.makedirs(save_dir, exist_ok=True)
            os.makedirs(os.path.join(save_dir, "left"), exist_ok=True)
            os.makedirs(os.path.join(save_dir, "right"), exist_ok=True)
        
        image_count = 0
        
        print(f"Starting stereo calibration capture...")
        print(f"Need {min_pairs} good stereo pairs with chessboard pattern")
        print(f"Looking for {self.chessboard_size[0]}x{self.chessboard_size[1]} internal corners")
        print(f"Instructions:")
        print(f"- Hold chessboard visible to BOTH cameras simultaneously")
        print(f"- Try different distances and angles")
        print(f"- Ensure good lighting for both cameras")
        print(f"- Both cameras must detect the pattern for capture")
        print(f"- Press 'q' to quit, 'r' to reset, 'd' to toggle debug")
        print(f"- Press 'p' to toggle preprocessing view")
        
        cv.namedWindow('Stereo Calibration - Left', cv.WINDOW_AUTOSIZE)
        cv.namedWindow('Stereo Calibration - Right', cv.WINDOW_AUTOSIZE)
        
        if self.show_preprocessing:
            cv.namedWindow('Preprocessed Left', cv.WINDOW_AUTOSIZE)
            cv.namedWindow('Preprocessed Right', cv.WINDOW_AUTOSIZE)
        
        while self.good_stereo_pairs < min_pairs and self.capturing:
            rclpy.spin_once(self, timeout_sec=0.001)
            
            with self.frame_lock:
                if self.current_frame_left is None or self.current_frame_right is None:
                    time.sleep(0.01)
                    continue
                
                frame_left = self.current_frame_left.copy()
                frame_right = self.current_frame_right.copy()
            
            # Convert to color for display
            display_left = cv.cvtColor(frame_left, cv.COLOR_GRAY2BGR)
            display_right = cv.cvtColor(frame_right, cv.COLOR_GRAY2BGR)
            
            # Try to detect chessboard in both images
            ret_left, corners_left, method_left = self.detect_chessboard_robust(frame_left)
            ret_right, corners_right, method_right = self.detect_chessboard_robust(frame_right)
            
            # Show preprocessing if enabled
            if self.show_preprocessing:
                processed_left = self.preprocess_image(frame_left)
                processed_right = self.preprocess_image(frame_right)
                cv.imshow('Preprocessed Left', processed_left)
                cv.imshow('Preprocessed Right', processed_right)
            
            # Draw status information
            status_text = f"Stereo pairs: {self.good_stereo_pairs}/{min_pairs}"
            cv.putText(display_left, status_text, (10, 30), 
                      cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv.putText(display_right, status_text, (10, 30), 
                      cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            current_time = time.time()
            
            # Check if pattern detected in BOTH cameras
            if ret_left and ret_right:
                # Draw corners on both images
                cv.drawChessboardCorners(display_left, self.chessboard_size, corners_left, ret_left)
                cv.drawChessboardCorners(display_right, self.chessboard_size, corners_right, ret_right)
                
                # Show detection methods
                cv.putText(display_left, f"Method: {method_left}", (10, 60), 
                          cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                cv.putText(display_right, f"Method: {method_right}", (10, 60), 
                          cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                # Check if enough time has passed since last capture
                if current_time - self.last_capture_time > self.capture_interval:
                    # Refine corner positions for both cameras
                    corners_left_refined = cv.cornerSubPix(
                        frame_left, corners_left, (11, 11), (-1, -1), self.criteria
                    )
                    corners_right_refined = cv.cornerSubPix(
                        frame_right, corners_right, (11, 11), (-1, -1), self.criteria
                    )
                    
                    # Store the points
                    self.objpoints.append(self.objp)
                    self.imgpoints_left.append(corners_left_refined)
                    self.imgpoints_right.append(corners_right_refined)
                    
                    self.good_stereo_pairs += 1
                    self.last_capture_time = current_time
                    
                    # Save images if requested
                    if save_images:
                        left_filename = os.path.join(save_dir, "left", f"left_{image_count:03d}.jpg")
                        right_filename = os.path.join(save_dir, "right", f"right_{image_count:03d}.jpg")
                        cv.imwrite(left_filename, display_left)
                        cv.imwrite(right_filename, display_right)
                        image_count += 1
                    
                    print(f"✓ Captured stereo pair {self.good_stereo_pairs}/{min_pairs}")
                    
                    # Flash green border on both images
                    cv.rectangle(display_left, (0, 0), 
                               (display_left.shape[1]-1, display_left.shape[0]-1), 
                               (0, 255, 0), 10)
                    cv.rectangle(display_right, (0, 0), 
                               (display_right.shape[1]-1, display_right.shape[0]-1), 
                               (0, 255, 0), 10)
                
                # Show successful detection
                cv.putText(display_left, "STEREO PATTERN DETECTED", (10, 90), 
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv.putText(display_right, "STEREO PATTERN DETECTED", (10, 90), 
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                # Show individual detection status
                left_status = "Pattern detected" if ret_left else "No pattern"
                right_status = "Pattern detected" if ret_right else "No pattern"
                
                left_color = (0, 255, 0) if ret_left else (0, 0, 255)
                right_color = (0, 255, 0) if ret_right else (0, 0, 255)
                
                cv.putText(display_left, f"Left: {left_status}", (10, 90), 
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, left_color, 2)
                cv.putText(display_right, f"Right: {right_status}", (10, 90), 
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, right_color, 2)
                
                if ret_left:
                    cv.drawChessboardCorners(display_left, self.chessboard_size, corners_left, ret_left)
                if ret_right:
                    cv.drawChessboardCorners(display_right, self.chessboard_size, corners_right, ret_right)
                
                # Show requirement
                cv.putText(display_left, "Need BOTH cameras to detect pattern", (10, 120), 
                          cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv.putText(display_right, "Need BOTH cameras to detect pattern", (10, 120), 
                          cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Show progress bars
            progress = self.good_stereo_pairs / min_pairs
            bar_width = 300
            bar_height = 20
            
            for display_frame in [display_left, display_right]:
                bar_x = (display_frame.shape[1] - bar_width) // 2
                bar_y = display_frame.shape[0] - 40
                
                cv.rectangle(display_frame, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height), 
                            (50, 50, 50), -1)
                cv.rectangle(display_frame, (bar_x, bar_y), 
                            (bar_x + int(bar_width * progress), bar_y + bar_height), 
                            (0, 255, 0), -1)
            
            cv.imshow('Stereo Calibration - Left', display_left)
            cv.imshow('Stereo Calibration - Right', display_right)
            
            key = cv.waitKey(1) & 0xFF
            if key == ord('q'):
                self.capturing = False
                break
            elif key == ord('r'):
                # Reset calibration
                self.objpoints = []
                self.imgpoints_left = []
                self.imgpoints_right = []
                self.good_stereo_pairs = 0
                print("Stereo calibration reset!")
            elif key == ord('d'):
                self.debug_mode = not self.debug_mode
                print(f"Debug mode: {'ON' if self.debug_mode else 'OFF'}")
            elif key == ord('p'):
                self.show_preprocessing = not self.show_preprocessing
                if self.show_preprocessing:
                    cv.namedWindow('Preprocessed Left', cv.WINDOW_AUTOSIZE)
                    cv.namedWindow('Preprocessed Right', cv.WINDOW_AUTOSIZE)
                else:
                    cv.destroyWindow('Preprocessed Left')
                    cv.destroyWindow('Preprocessed Right')
                print(f"Preprocessing view: {'ON' if self.show_preprocessing else 'OFF'}")
        
        cv.destroyAllWindows()
        
        if self.good_stereo_pairs >= min_pairs:
            print(f"✓ Successfully captured {self.good_stereo_pairs} stereo pairs")
            return True
        else:
            print(f"✗ Only captured {self.good_stereo_pairs} pairs, need {min_pairs}")
            return False
    
    def calibrate_individual_cameras(self, image_size):
        """Calibrate each camera individually first"""
        if len(self.objpoints) == 0:
            print("Error: No calibration data available")
            return False
        
        print("Calibrating individual cameras...")
        
        # Calibrate left camera
        print("Calibrating left camera...")
        ret_left, mtx_left, dist_left, rvecs_left, tvecs_left = cv.calibrateCamera(
            self.objpoints, self.imgpoints_left, image_size, None, None
        )
        
        # Calibrate right camera
        print("Calibrating right camera...")
        ret_right, mtx_right, dist_right, rvecs_right, tvecs_right = cv.calibrateCamera(
            self.objpoints, self.imgpoints_right, image_size, None, None
        )
        
        if ret_left and ret_right:
            self.camera_matrix_left = mtx_left
            self.dist_coeffs_left = dist_left
            self.camera_matrix_right = mtx_right
            self.dist_coeffs_right = dist_right
            
            print("✓ Individual camera calibrations successful!")
            return True
        else:
            print("✗ Individual camera calibrations failed")
            return False
    
    def calibrate_stereo_cameras(self, image_size):
        """Perform stereo calibration to find relative camera positions"""
        if not self.calibrate_individual_cameras(image_size):
            return False
        
        print("Performing stereo calibration...")
        
        # Stereo calibration
        flags = cv.CALIB_FIX_INTRINSIC
        
        ret, mtx_left, dist_left, mtx_right, dist_right, R, T, E, F = cv.stereoCalibrate(
            self.objpoints, self.imgpoints_left, self.imgpoints_right,
            self.camera_matrix_left, self.dist_coeffs_left,
            self.camera_matrix_right, self.dist_coeffs_right,
            image_size, criteria=self.criteria, flags=flags
        )
        
        if ret:
            self.R = R  # Rotation matrix
            self.T = T  # Translation vector
            self.E = E  # Essential matrix
            self.F = F  # Fundamental matrix
            
            # Calculate stereo calibration error
            self.stereo_error = self.calculate_stereo_error()
            
            print("✓ Stereo calibration successful!")
            print(f"Stereo calibration error: {self.stereo_error:.4f} pixels")
            print(f"Baseline (distance between cameras): {np.linalg.norm(T):.2f}mm")
            print(f"Translation vector: {T.ravel()}")
            
            # Compute rectification transforms
            self.compute_rectification(image_size)
            
            return True
        else:
            print("✗ Stereo calibration failed")
            return False
    
    def calculate_stereo_error(self):
        """Calculate stereo calibration reprojection error"""
        mean_error = 0
        for i in range(len(self.objpoints)):
            # Project points for left camera
            imgpoints2_left, _ = cv.projectPoints(
                self.objpoints[i], np.zeros((3,1)), np.zeros((3,1)), 
                self.camera_matrix_left, self.dist_coeffs_left
            )
            
            # Project points for right camera (considering stereo transformation)
            # For simplicity, we'll use the individual camera errors
            imgpoints2_right, _ = cv.projectPoints(
                self.objpoints[i], np.zeros((3,1)), np.zeros((3,1)), 
                self.camera_matrix_right, self.dist_coeffs_right
            )
            
            error_left = cv.norm(self.imgpoints_left[i], imgpoints2_left, cv.NORM_L2) / len(imgpoints2_left)
            error_right = cv.norm(self.imgpoints_right[i], imgpoints2_right, cv.NORM_L2) / len(imgpoints2_right)
            mean_error += (error_left + error_right) / 2
        
        return mean_error / len(self.objpoints)
    
    def compute_rectification(self, image_size):
        """Compute stereo rectification transforms"""
        print("Computing stereo rectification...")
        
        # Stereo rectification
        self.R1, self.R2, self.P1, self.P2, self.Q, roi_left, roi_right = cv.stereoRectify(
            self.camera_matrix_left, self.dist_coeffs_left,
            self.camera_matrix_right, self.dist_coeffs_right,
            image_size, self.R, self.T, alpha=1
        )
        
        self.roi_left = roi_left
        self.roi_right = roi_right
        
        # Compute rectification maps
        self.map1_left, self.map2_left = cv.initUndistortRectifyMap(
            self.camera_matrix_left, self.dist_coeffs_left, self.R1, self.P1, image_size, cv.CV_16SC2
        )
        
        self.map1_right, self.map2_right = cv.initUndistortRectifyMap(
            self.camera_matrix_right, self.dist_coeffs_right, self.R2, self.P2, image_size, cv.CV_16SC2
        )
        
        print("✓ Stereo rectification computed successfully!")
    
    def save_stereo_calibration(self, filename="stereo_calibration.json"):
        """Save stereo calibration results to file"""
        if self.camera_matrix_left is None or self.R is None:
            print("Error: No calibration data to save")
            return False
        
        calibration_data = {
            "camera_matrix_left": self.camera_matrix_left.tolist(),
            "distortion_coefficients_left": self.dist_coeffs_left.tolist(),
            "camera_matrix_right": self.camera_matrix_right.tolist(),
            "distortion_coefficients_right": self.dist_coeffs_right.tolist(),
            "rotation_matrix": self.R.tolist(),
            "translation_vector": self.T.tolist(),
            "essential_matrix": self.E.tolist(),
            "fundamental_matrix": self.F.tolist(),
            "rectification_left": self.R1.tolist(),
            "rectification_right": self.R2.tolist(),
            "projection_left": self.P1.tolist(),
            "projection_right": self.P2.tolist(),
            "disparity_to_depth_matrix": self.Q.tolist(),
            "baseline_distance_mm": float(np.linalg.norm(self.T)),
            "stereo_calibration_error": float(self.stereo_error),
            "chessboard_size": self.chessboard_size,
            "square_size": self.square_size,
            "calibration_date": datetime.now().isoformat()
        }
        
        with open(filename, 'w') as f:
            json.dump(calibration_data, f, indent=2)
        
        print(f"✓ Stereo calibration saved to {filename}")
        return True
    
    def test_stereo_rectification(self):
        """Test stereo rectification with live camera feed"""
        if self.map1_left is None:
            print("Error: Stereo calibration not completed")
            return
        
        print("Testing stereo rectification - Press 'q' to quit")
        print("Green lines show that the images are properly rectified")
        
        cv.namedWindow('Stereo Rectified', cv.WINDOW_AUTOSIZE)
        
        while True:
            rclpy.spin_once(self, timeout_sec=0.001)
            
            with self.frame_lock:
                if self.current_frame_left is None or self.current_frame_right is None:
                    time.sleep(0.01)
                    continue
                
                frame_left = self.current_frame_left.copy()
                frame_right = self.current_frame_right.copy()
            
            # Rectify images
            rectified_left = cv.remap(frame_left, self.map1_left, self.map2_left, cv.INTER_LINEAR)
            rectified_right = cv.remap(frame_right, self.map1_right, self.map2_right, cv.INTER_LINEAR)
            
            # Convert to color for display
            rectified_left_color = cv.cvtColor(rectified_left, cv.COLOR_GRAY2BGR)
            rectified_right_color = cv.cvtColor(rectified_right, cv.COLOR_GRAY2BGR)
            
            # Combine images side by side
            combined = np.hstack((rectified_left_color, rectified_right_color))
            
            # Draw horizontal lines to verify rectification
            h, w = combined.shape[:2]
            for i in range(0, h, 50):
                cv.line(combined, (0, i), (w, i), (0, 255, 0), 1)
            
            # Add labels
            cv.putText(combined, "Left Rectified", (10, 30), 
                      cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv.putText(combined, "Right Rectified", (w//2 + 10, 30), 
                      cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            cv.imshow('Stereo Rectified', combined)
            
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
        
        cv.destroyAllWindows()

def main():
    """Main function to run stereo camera calibration"""
    print("=== ROS2 Stereo Camera Calibration ===")
    print("This tool calibrates a stereo camera setup!")
    print()
    
    # Get user input for chessboard configuration
    print("Chessboard configuration:")
    try:
        width = int(input("Enter number of internal corners width (default 9): ") or "9")
        height = int(input("Enter number of internal corners height (default 7): ") or "7")
        square_size = float(input("Enter square size in mm (default 25.0): ") or "25.0")
    except ValueError:
        print("Invalid input, using defaults")
        width, height, square_size = 9, 7, 25.0
    
    print(f"Using chessboard: {width}x{height} corners, {square_size}mm squares")
    print()
    
    print("Make sure your camera nodes are running:")
    print("Left camera:  ros2 run camera_ros camera_node --ros-args -r image_raw:=camera_left/image_raw ...")
    print("Right camera: ros2 run camera_ros camera_node --ros-args -r image_raw:=camera_right/image_raw ...")
    print()
    
    # Initialize ROS2
    rclpy.init()
    
    # Create calibrator
    calibrator = ROS2StereoCameraCalibrator(chessboard_size=(width, height), square_size=square_size)
    
    try:
        # Step 1: Capture stereo calibration images
        if calibrator.capture_stereo_calibration_images(min_pairs=15):
            
            # Step 2: Perform stereo calibration
            # Get image size from current frame
            with calibrator.frame_lock:
                if calibrator.current_frame_left is not None:
                    h, w = calibrator.current_frame_left.shape[:2]
                    image_size = (w, h)
                    
                    if calibrator.calibrate_stereo_cameras(image_size):
                        
                        # Step 3: Save calibration
                        calibrator.save_stereo_calibration()
                        
                        # Step 4: Test rectification
                        response = input("Test stereo rectification with live cameras? (y/n): ")
                        if response.lower() == 'y':
                            calibrator.test_stereo_rectification()
                        
                        print("Stereo calibration completed successfully!")
                        print(f"Baseline distance: {np.linalg.norm(calibrator.T):.2f}mm")
                        print("You can now use the stereo_calibration.json file for depth estimation!")
                    else:
                        print("Stereo calibration failed!")
                else:
                    print("Could not get image size from cameras")
        else:
            print("Failed to capture enough stereo calibration pairs")
    
    except KeyboardInterrupt:
        print("\nStereo calibration interrupted by user")
    
    finally:
        # Cleanup
        calibrator.destroy_node()
        rclpy.shutdown()
        cv.destroyAllWindows()

if __name__ == "__main__":
    main()