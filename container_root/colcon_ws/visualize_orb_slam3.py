#!/usr/bin/env python3
"""
ORB-SLAM3 Map Visualization Script
This script subscribes to ORB-SLAM3 topics and provides visualization information
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from slam_msgs.msg import SlamInfo, MapData
import numpy as np

class ORBSLAMVisualizer(Node):
    def __init__(self):
        super().__init__('orb_slam3_visualizer')
        
        # Subscribers
        self.map_points_sub = self.create_subscription(
            PointCloud2, '/map_points', self.map_points_callback, 10)
        
        self.robot_pose_sub = self.create_subscription(
            PoseStamped, '/robot_pose_slam', self.robot_pose_callback, 10)
        
        self.slam_info_sub = self.create_subscription(
            SlamInfo, '/slam_info', self.slam_info_callback, 10)
        
        self.map_data_sub = self.create_subscription(
            MapData, '/map_data', self.map_data_callback, 10)
        
        # State variables
        self.map_points_count = 0
        self.robot_pose = None
        self.slam_info = None
        self.map_data = None
        self.last_update = self.get_clock().now()
        
        # Timer for status updates
        self.status_timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('ORB-SLAM3 Visualizer started - monitoring SLAM topics')
        self.get_logger().info('=== ORB-SLAM3 MAP VISUALIZATION ===')
        self.get_logger().info('Monitoring topics:')
        self.get_logger().info('  - /map_points (PointCloud2)')
        self.get_logger().info('  - /robot_pose_slam (PoseStamped)')
        self.get_logger().info('  - /slam_info (SlamInfo)')
        self.get_logger().info('  - /map_data (MapData)')
        self.get_logger().info('=====================================')

    def map_points_callback(self, msg):
        # Count points in the point cloud
        point_step = msg.point_step
        if point_step > 0:
            self.map_points_count = len(msg.data) // point_step
        else:
            self.map_points_count = 0
        self.last_update = self.get_clock().now()

    def robot_pose_callback(self, msg):
        self.robot_pose = msg
        self.last_update = self.get_clock().now()

    def slam_info_callback(self, msg):
        self.slam_info = msg
        self.last_update = self.get_clock().now()

    def map_data_callback(self, msg):
        self.map_data = msg
        self.last_update = self.get_clock().now()

    def print_status(self):
        current_time = self.get_clock().now()
        time_since_update = (current_time - self.last_update).nanoseconds / 1e9
        
        self.get_logger().info('=== ORB-SLAM3 STATUS ===')
        
        if time_since_update > 5.0:
            self.get_logger().warn('⚠ No updates received for {:.1f} seconds'.format(time_since_update))
            self.get_logger().warn('ORB-SLAM3 may not be initialized yet or camera not moving')
        else:
            self.get_logger().info('✓ Receiving updates (last: {:.1f}s ago)'.format(time_since_update))
        
        # SLAM Info
        if self.slam_info:
            self.get_logger().info('Maps: {}, Tracking freq: {:.1f} Hz'.format(
                self.slam_info.num_maps, self.slam_info.tracking_frequency))
        else:
            self.get_logger().warn('No SLAM info received')
        
        # Map Points
        if self.map_points_count > 0:
            self.get_logger().info('✓ Map Points: {} points in map'.format(self.map_points_count))
            if self.map_points_count > 100:
                self.get_logger().info('🎉 SLAM INITIALIZED! Map has sufficient points')
            else:
                self.get_logger().warn('Map has few points - may still be initializing')
        else:
            self.get_logger().warn('❌ No map points - SLAM not initialized')
        
        # Robot Pose
        if self.robot_pose:
            pos = self.robot_pose.pose.position
            self.get_logger().info('Robot Position: x={:.2f}, y={:.2f}, z={:.2f}'.format(
                pos.x, pos.y, pos.z))
        else:
            self.get_logger().warn('No robot pose received')
        
        # Map Data
        if self.map_data:
            keyframes = len(self.map_data.graph.poses) if hasattr(self.map_data, 'graph') else 0
            self.get_logger().info('Keyframes: {}'.format(keyframes))
        else:
            self.get_logger().warn('No map data received')
        
        self.get_logger().info('========================')
        
        # Provide initialization tips if not initialized
        if self.map_points_count == 0:
            self.get_logger().info('💡 INITIALIZATION TIPS:')
            self.get_logger().info('  1. Move camera SLOWLY in different directions')
            self.get_logger().info('  2. Point at textured surfaces (avoid blank walls)')
            self.get_logger().info('  3. Ensure good lighting')
            self.get_logger().info('  4. Be patient - initialization can take 30+ seconds')

def main(args=None):
    rclpy.init(args=args)
    
    visualizer = ORBSLAMVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
