#!/usr/bin/env python3
"""

"""

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

class ROS2CameraCalibrator(Node):
    def __init__(self, chessboard_size=(9, 7), square_size=25.0):
        """
        Initialize the ROS2 camera calibrator
        
        Args:
            chessboard_size: Tuple of (width, height) of internal corners
            square_size: Size of chessboard square in mm (for real-world scaling)
        """
        super().__init__('camera_calibrator')
        
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
        
        # Arrays to store object points and image points
        self.objpoints = []  # 3D points in real world space
        self.imgpoints = []  # 2D points in image plane
        
        # Calibration results
        self.camera_matrix = None
        self.dist_coeffs = None
        self.calibration_error = None
        
        # Image capture variables
        self.current_frame = None
        self.frame_lock = threading.Lock()
        self.capturing = False
        self.good_images = 0
        self.min_images = 15
        self.last_capture_time = 0
        self.capture_interval = 2.0
        
        # Debug variables
        self.debug_mode = True
        self.show_preprocessing = False
        
        # Create image subscriber
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Default topic, can be changed
            self.image_callback,
            10
        )
        
        print("ROS2 Camera Calibrator initialized")
        print("Waiting for images on topic: /camera/image_raw")
    
    def image_callback(self, msg):
        """Callback for receiving camera images"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            
            with self.frame_lock:
                self.current_frame = cv_image.copy()
                
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
    
    def preprocess_image(self, image):
        """Enhanced image preprocessing for better chessboard detection"""
        # Start with original image
        processed = image.copy()
        
        # Apply Gaussian blur to reduce noise
        processed = cv.GaussianBlur(processed, (5, 5), 0)
        
        # Enhance contrast using CLAHE (Contrast Limited Adaptive Histogram Equalization)
        clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        processed = clahe.apply(processed)
        
        # Optional: Apply bilateral filter to smooth while preserving edges
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
        
        # Method 4: Try different chessboard sizes (in case user miscounted)
        alternative_sizes = [
            (8, 6), (7, 5), (10, 8), (6, 4), (9, 6), (8, 7)
        ]
        
        ret4, corners4 = False, None
        detected_size = None
        
        for alt_size in alternative_sizes:
            if alt_size != self.chessboard_size:
                ret_alt, corners_alt = cv.findChessboardCorners(gray, alt_size, None)
                if ret_alt:
                    ret4, corners4 = ret_alt, corners_alt
                    detected_size = alt_size
                    break
        
        # Choose the best result
        results = [(ret1, corners1, "Standard"), 
                  (ret2, corners2, "Preprocessed"), 
                  (ret3, corners3, "Adaptive"),
                  (ret4, corners4, f"Alt size {detected_size}" if detected_size else "Alt size")]
        
        for ret, corners, method in results:
            if ret:
                if self.debug_mode:
                    print(f"Chessboard detected using: {method}")
                return ret, corners, method
        
        return False, None, "None"
    
    def wait_for_camera(self, timeout=10.0):
        """Wait for camera images to start arriving"""
        print("Waiting for camera images...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            with self.frame_lock:
                if self.current_frame is not None:
                    print("✓ Camera images detected!")
                    h, w = self.current_frame.shape[:2]
                    print(f"Image size: {w}x{h}")
                    return True
            
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        print("✗ Timeout waiting for camera images")
        print("Make sure your camera node is running and publishing to /camera/image_raw")
        return False
    
    def capture_calibration_images(self, min_images=15, save_images=True):
        """
        Automatically capture calibration images from ROS2 camera with enhanced detection
        """
        self.min_images = min_images
        self.good_images = 0
        self.capturing = True
        
        # Wait for camera to be available
        if not self.wait_for_camera():
            return False
        
        # Create directory for saving images
        if save_images:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_dir = f"calibration_images_{timestamp}"
            os.makedirs(save_dir, exist_ok=True)
        
        image_count = 0
        
        print(f"Starting automatic calibration capture...")
        print(f"Need {min_images} good images with chessboard pattern")
        print(f"Looking for {self.chessboard_size[0]}x{self.chessboard_size[1]} internal corners")
        print(f"Instructions:")
        print(f"- Hold chessboard in front of camera")
        print(f"- Try different distances (30-100cm from camera)")
        print(f"- Try different angles and orientations")
        print(f"- Ensure good, even lighting")
        print(f"- Press 'q' to quit, 'r' to reset, 'd' to toggle debug")
        print(f"- Press 'p' to toggle preprocessing view")
        print(f"- Images are captured automatically when pattern is detected")
        
        cv.namedWindow('Camera Calibration', cv.WINDOW_AUTOSIZE)
        if self.show_preprocessing:
            cv.namedWindow('Preprocessed', cv.WINDOW_AUTOSIZE)
        
        while self.good_images < min_images and self.capturing:
            # Spin ROS2 to get new images
            rclpy.spin_once(self, timeout_sec=0.001)
            
            with self.frame_lock:
                if self.current_frame is None:
                    time.sleep(0.01)
                    continue
                    
                frame = self.current_frame.copy()
            
            # Convert to color for display
            display_frame = cv.cvtColor(frame, cv.COLOR_GRAY2BGR)
            
            # Try robust chessboard detection
            ret_chess, corners, method = self.detect_chessboard_robust(frame)
            
            # Show preprocessing if enabled
            if self.show_preprocessing:
                processed = self.preprocess_image(frame)
                cv.imshow('Preprocessed', processed)
            
            # Draw status information
            status_text = f"Good images: {self.good_images}/{min_images}"
            cv.putText(display_frame, status_text, (10, 30), 
                      cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # Show detection method
            if method != "None":
                method_text = f"Method: {method}"
                cv.putText(display_frame, method_text, (10, 60), 
                          cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            current_time = time.time()
            
            if ret_chess:
                # Draw the corners
                cv.drawChessboardCorners(display_frame, self.chessboard_size, corners, ret_chess)
                
                # Check if enough time has passed since last capture
                if current_time - self.last_capture_time > self.capture_interval:
                    # Refine corner positions
                    corners_refined = cv.cornerSubPix(
                        frame, corners, (11, 11), (-1, -1), self.criteria
                    )
                    
                    # Store the points
                    self.objpoints.append(self.objp)
                    self.imgpoints.append(corners_refined)
                    
                    self.good_images += 1
                    self.last_capture_time = current_time
                    
                    # Save image if requested
                    if save_images:
                        filename = os.path.join(save_dir, f"calib_{image_count:03d}.jpg")
                        cv.imwrite(filename, display_frame)
                        image_count += 1
                    
                    # Visual feedback
                    print(f"✓ Captured image {self.good_images}/{min_images} using {method}")
                    
                    # Flash green border
                    cv.rectangle(display_frame, (0, 0), 
                               (display_frame.shape[1]-1, display_frame.shape[0]-1), 
                               (0, 255, 0), 10)
                
                # Show pattern detection status
                cv.putText(display_frame, "Pattern detected - Hold steady", (10, 90), 
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Show corner count
                corner_text = f"Corners: {len(corners)}/{self.chessboard_size[0]*self.chessboard_size[1]}"
                cv.putText(display_frame, corner_text, (10, 120), 
                          cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv.putText(display_frame, "No pattern detected", (10, 90), 
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Show troubleshooting tips
                tips = [
                    "- Check lighting (avoid shadows/glare)",
                    "- Try different distance/angle",
                    "- Ensure chessboard is flat",
                    f"- Verify {self.chessboard_size[0]}x{self.chessboard_size[1]} pattern"
                ]
                
                for i, tip in enumerate(tips):
                    cv.putText(display_frame, tip, (10, 120 + i*25), 
                              cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            # Show progress bar
            progress = self.good_images / min_images
            bar_width = 400
            bar_height = 20
            bar_x = (display_frame.shape[1] - bar_width) // 2
            bar_y = display_frame.shape[0] - 40
            
            cv.rectangle(display_frame, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height), 
                        (50, 50, 50), -1)
            cv.rectangle(display_frame, (bar_x, bar_y), 
                        (bar_x + int(bar_width * progress), bar_y + bar_height), 
                        (0, 255, 0), -1)
            
            cv.imshow('Camera Calibration', display_frame)
            
            key = cv.waitKey(1) & 0xFF
            if key == ord('q'):
                self.capturing = False
                break
            elif key == ord('r'):
                # Reset calibration
                self.objpoints = []
                self.imgpoints = []
                self.good_images = 0
                print("Calibration reset!")
            elif key == ord('d'):
                # Toggle debug mode
                self.debug_mode = not self.debug_mode
                print(f"Debug mode: {'ON' if self.debug_mode else 'OFF'}")
            elif key == ord('p'):
                # Toggle preprocessing view
                self.show_preprocessing = not self.show_preprocessing
                if self.show_preprocessing:
                    cv.namedWindow('Preprocessed', cv.WINDOW_AUTOSIZE)
                else:
                    cv.destroyWindow('Preprocessed')
                print(f"Preprocessing view: {'ON' if self.show_preprocessing else 'OFF'}")
        
        cv.destroyAllWindows()
        
        if self.good_images >= min_images:
            print(f"✓ Successfully captured {self.good_images} calibration images")
            return True
        else:
            print(f"✗ Only captured {self.good_images} images, need {min_images}")
            return False
    
    def calibrate_camera(self, image_size):
        """
        Perform camera calibration using collected points
        
        Args:
            image_size: Tuple of (width, height) of images
        """
        if len(self.objpoints) == 0:
            print("Error: No calibration data available")
            return False
        
        print("Performing camera calibration...")
        
        # Calibrate camera
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
            self.objpoints, self.imgpoints, image_size, None, None
        )
        
        if ret:
            self.camera_matrix = mtx
            self.dist_coeffs = dist
            
            # Calculate reprojection error
            self.calibration_error = self.calculate_reprojection_error(rvecs, tvecs)
            
            print("✓ Camera calibration successful!")
            print(f"Reprojection error: {self.calibration_error:.4f} pixels")
            print(f"Camera matrix:\n{self.camera_matrix}")
            print(f"Distortion coefficients: {self.dist_coeffs.ravel()}")
            
            return True
        else:
            print("✗ Camera calibration failed")
            return False
    
    def calculate_reprojection_error(self, rvecs, tvecs):
        """Calculate the reprojection error"""
        mean_error = 0
        for i in range(len(self.objpoints)):
            imgpoints2, _ = cv.projectPoints(
                self.objpoints[i], rvecs[i], tvecs[i], 
                self.camera_matrix, self.dist_coeffs
            )
            error = cv.norm(self.imgpoints[i], imgpoints2, cv.NORM_L2) / len(imgpoints2)
            mean_error += error
        
        return mean_error / len(self.objpoints)
    
    def save_calibration(self, filename="camera_calibration.json"):
        """Save calibration results to file"""
        if self.camera_matrix is None:
            print("Error: No calibration data to save")
            return False
        
        calibration_data = {
            "camera_matrix": self.camera_matrix.tolist(),
            "distortion_coefficients": self.dist_coeffs.tolist(),
            "reprojection_error": float(self.calibration_error),
            "chessboard_size": self.chessboard_size,
            "square_size": self.square_size,
            "calibration_date": datetime.now().isoformat()
        }
        
        with open(filename, 'w') as f:
            json.dump(calibration_data, f, indent=2)
        
        print(f"✓ Calibration saved to {filename}")
        return True
    
    def load_calibration(self, filename="camera_calibration.json"):
        """Load calibration results from file"""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            self.camera_matrix = np.array(data["camera_matrix"])
            self.dist_coeffs = np.array(data["distortion_coefficients"])
            self.calibration_error = data["reprojection_error"]
            
            print(f"✓ Calibration loaded from {filename}")
            print(f"Reprojection error: {self.calibration_error:.4f} pixels")
            return True
        except Exception as e:
            print(f"✗ Error loading calibration: {e}")
            return False
    
    def undistort_image(self, image):
        """Undistort an image using calibration parameters"""
        if self.camera_matrix is None:
            print("Error: Camera not calibrated")
            return None
        
        h, w = image.shape[:2]
        
        # Get optimal new camera matrix
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
        )
        
        # Undistort
        dst = cv.undistort(image, self.camera_matrix, self.dist_coeffs, None, newcameramtx)
        
        # Crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        
        return dst
    
    def test_undistortion(self):
        """Test undistortion with live camera feed"""
        if self.camera_matrix is None:
            print("Error: Camera not calibrated")
            return
        
        print("Testing undistortion - Press 'q' to quit")
        
        cv.namedWindow('Undistortion Test', cv.WINDOW_AUTOSIZE)
        
        while True:
            rclpy.spin_once(self, timeout_sec=0.001)
            
            with self.frame_lock:
                if self.current_frame is None:
                    time.sleep(0.01)
                    continue
                frame = self.current_frame.copy()
            
            # Convert to color for display
            frame_color = cv.cvtColor(frame, cv.COLOR_GRAY2BGR)
            
            # Undistort frame
            undistorted = self.undistort_image(frame)
            
            if undistorted is not None:
                # Convert undistorted to color
                undistorted_color = cv.cvtColor(undistorted, cv.COLOR_GRAY2BGR)
                
                # Resize for display if needed
                h1, w1 = frame_color.shape[:2]
                h2, w2 = undistorted_color.shape[:2]
                
                if h1 != h2 or w1 != w2:
                    undistorted_color = cv.resize(undistorted_color, (w1, h1))
                
                # Show side by side
                combined = np.hstack((frame_color, undistorted_color))
                
                # Add labels
                cv.putText(combined, "Original", (10, 30), 
                          cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv.putText(combined, "Undistorted", (w1 + 10, 30), 
                          cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                cv.imshow('Undistortion Test', combined)
            else:
                cv.imshow('Undistortion Test', frame_color)
            
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
        
        cv.destroyAllWindows()

def main():
    """Main function to run ROS2 camera calibration"""
    print("=== Enhanced ROS2 Camera Calibration ===")
    print("This version includes enhanced chessboard detection!")
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
    
    # Initialize ROS2
    rclpy.init()
    
    # Create calibrator
    calibrator = ROS2CameraCalibrator(chessboard_size=(width, height), square_size=square_size)
    
    try:
        # Step 1: Capture calibration images
        if calibrator.capture_calibration_images(min_images=12):
            
            # Step 2: Perform calibration
            # Get image size from current frame
            with calibrator.frame_lock:
                if calibrator.current_frame is not None:
                    h, w = calibrator.current_frame.shape[:2]
                    image_size = (w, h)
                    
                    if calibrator.calibrate_camera(image_size):
                        
                        # Step 3: Save calibration
                        calibrator.save_calibration()
                        
                        # Step 4: Test undistortion
                        response = input("Test undistortion with live camera? (y/n): ")
                        if response.lower() == 'y':
                            calibrator.test_undistortion()
                        
                        print("Calibration completed successfully!")
                    else:
                        print("Calibration failed!")
                else:
                    print("Could not get image size from camera")
        else:
            print("Failed to capture enough calibration images")
    
    except KeyboardInterrupt:
        print("\nCalibration interrupted by user")
    
    finally:
        # Cleanup
        calibrator.destroy_node()
        rclpy.shutdown()
        cv.destroyAllWindows()

if __name__ == "__main__":
    main()