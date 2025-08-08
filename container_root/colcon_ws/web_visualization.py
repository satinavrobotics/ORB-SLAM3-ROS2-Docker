#!/usr/bin/env python3
"""
ORB-SLAM3 Web-based Visualization
This script creates a web interface to visualize ORB-SLAM3 data
Works without X11 display - perfect for headless systems
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from slam_msgs.msg import SlamInfo, MapData
import numpy as np
import json
import struct
import threading
import time
from collections import deque
from http.server import HTTPServer, SimpleHTTPRequestHandler
import socketserver
import os

class ORBSLAMWebVisualizer(Node):
    def __init__(self):
        super().__init__('orb_slam3_web_visualizer')
        
        # Data storage
        self.map_points = []
        self.trajectory = deque(maxlen=1000)
        self.current_pose = None
        self.slam_info = None
        self.is_initialized = False
        self.last_update_time = time.time()
        self.total_map_points = 0
        self.total_keyframes = 0
        
        # Subscribers
        self.map_points_sub = self.create_subscription(
            PointCloud2, '/map_points', self.map_points_callback, 10)
        
        self.robot_pose_sub = self.create_subscription(
            PoseStamped, '/robot_pose_slam', self.robot_pose_callback, 10)
        
        self.slam_info_sub = self.create_subscription(
            SlamInfo, '/slam_info', self.slam_info_callback, 10)
        
        self.map_data_sub = self.create_subscription(
            MapData, '/map_data', self.map_data_callback, 10)
        
        # Create web files
        self.create_web_files()
        
        # Start web server in separate thread
        self.web_thread = threading.Thread(target=self.start_web_server, daemon=True)
        self.web_thread.start()
        
        # Data update timer
        self.timer = self.create_timer(1.0, self.update_data_file)
        
        self.get_logger().info('🌐 ORB-SLAM3 Web Visualizer started!')
        self.get_logger().info('🔗 Open http://localhost:8080 in your browser')
        self.get_logger().info('📊 Real-time visualization available via web interface')

    def parse_pointcloud2(self, cloud_msg):
        """Parse PointCloud2 message to extract 3D points"""
        points = []
        
        point_step = cloud_msg.point_step
        data = cloud_msg.data
        
        # Find field offsets
        x_offset = y_offset = z_offset = None
        for field in cloud_msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
        
        if x_offset is None or y_offset is None or z_offset is None:
            return points
        
        # Extract points (sample every 10th point for performance)
        for i in range(0, len(data), point_step * 10):
            if i + 12 <= len(data):
                try:
                    x = struct.unpack('f', data[i + x_offset:i + x_offset + 4])[0]
                    y = struct.unpack('f', data[i + y_offset:i + y_offset + 4])[0]
                    z = struct.unpack('f', data[i + z_offset:i + z_offset + 4])[0]
                    
                    if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                        points.append([float(x), float(y), float(z)])
                except:
                    continue
        
        return points

    def map_points_callback(self, msg):
        """Handle map points updates"""
        try:
            self.map_points = self.parse_pointcloud2(msg)
            self.total_map_points = len(self.map_points) * 10  # Account for sampling
            self.last_update_time = time.time()
            
            if self.total_map_points > 100 and not self.is_initialized:
                self.is_initialized = True
                self.get_logger().info('🎉 ORB-SLAM3 INITIALIZED! Map contains ~{} points'.format(self.total_map_points))
                
        except Exception as e:
            self.get_logger().error('Error parsing map points: {}'.format(str(e)))

    def robot_pose_callback(self, msg):
        """Handle robot pose updates"""
        try:
            self.current_pose = msg
            pos = msg.pose.position
            self.trajectory.append([float(pos.x), float(pos.y), float(pos.z)])
            self.last_update_time = time.time()
        except Exception as e:
            self.get_logger().error('Error processing robot pose: {}'.format(str(e)))

    def slam_info_callback(self, msg):
        """Handle SLAM info updates"""
        self.slam_info = msg
        self.last_update_time = time.time()

    def map_data_callback(self, msg):
        """Handle map data updates"""
        try:
            if hasattr(msg, 'graph') and hasattr(msg.graph, 'poses'):
                self.total_keyframes = len(msg.graph.poses)
            self.last_update_time = time.time()
        except Exception as e:
            self.get_logger().error('Error processing map data: {}'.format(str(e)))

    def update_data_file(self):
        """Update JSON data file for web interface"""
        try:
            data = {
                'timestamp': time.time(),
                'is_initialized': self.is_initialized,
                'total_map_points': self.total_map_points,
                'total_keyframes': self.total_keyframes,
                'trajectory_length': len(self.trajectory),
                'last_update_time': self.last_update_time,
                'map_points': self.map_points[:500],  # Limit for performance
                'trajectory': list(self.trajectory),
                'current_pose': {
                    'x': float(self.current_pose.pose.position.x) if self.current_pose else 0,
                    'y': float(self.current_pose.pose.position.y) if self.current_pose else 0,
                    'z': float(self.current_pose.pose.position.z) if self.current_pose else 0
                } if self.current_pose else None,
                'slam_info': {
                    'tracking_frequency': float(self.slam_info.tracking_frequency) if self.slam_info else 0,
                    'num_maps': int(self.slam_info.num_maps) if self.slam_info else 0
                } if self.slam_info else None
            }
            
            with open('/tmp/orb_slam_data.json', 'w') as f:
                json.dump(data, f)
                
        except Exception as e:
            self.get_logger().error('Error updating data file: {}'.format(str(e)))

    def create_web_files(self):
        """Create HTML and JavaScript files for web visualization"""
        html_content = '''<!DOCTYPE html>
<html>
<head>
    <title>ORB-SLAM3 Visualization</title>
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        .container { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
        .plot { height: 400px; border: 1px solid #ccc; }
        .status { background: #f0f0f0; padding: 15px; border-radius: 5px; }
        .initialized { background: #d4edda; }
        .initializing { background: #fff3cd; }
        .not-initialized { background: #f8d7da; }
        h1 { text-align: center; color: #333; }
        .stats { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 10px; }
        .stat-box { background: white; padding: 10px; border-radius: 5px; text-align: center; }
    </style>
</head>
<body>
    <h1>🚀 ORB-SLAM3 Real-time Visualization</h1>
    
    <div id="status" class="status">
        <h3>System Status: <span id="status-text">Loading...</span></h3>
        <div class="stats">
            <div class="stat-box">
                <strong>Map Points</strong><br>
                <span id="map-points">0</span>
            </div>
            <div class="stat-box">
                <strong>Keyframes</strong><br>
                <span id="keyframes">0</span>
            </div>
            <div class="stat-box">
                <strong>Trajectory Points</strong><br>
                <span id="trajectory-length">0</span>
            </div>
            <div class="stat-box">
                <strong>Tracking Freq</strong><br>
                <span id="tracking-freq">0</span> Hz
            </div>
        </div>
    </div>
    
    <div class="container">
        <div id="plot3d" class="plot"></div>
        <div id="plot2d" class="plot"></div>
    </div>
    
    <script>
        let plot3dDiv = document.getElementById('plot3d');
        let plot2dDiv = document.getElementById('plot2d');
        
        function updateVisualization() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    // Update status
                    const statusDiv = document.getElementById('status');
                    const statusText = document.getElementById('status-text');
                    
                    if (data.is_initialized) {
                        statusDiv.className = 'status initialized';
                        statusText.textContent = '✅ INITIALIZED';
                    } else if (data.total_map_points > 0) {
                        statusDiv.className = 'status initializing';
                        statusText.textContent = '🔄 INITIALIZING';
                    } else {
                        statusDiv.className = 'status not-initialized';
                        statusText.textContent = '❌ NOT INITIALIZED';
                    }
                    
                    // Update statistics
                    document.getElementById('map-points').textContent = data.total_map_points;
                    document.getElementById('keyframes').textContent = data.total_keyframes;
                    document.getElementById('trajectory-length').textContent = data.trajectory_length;
                    document.getElementById('tracking-freq').textContent = 
                        data.slam_info ? data.slam_info.tracking_frequency.toFixed(1) : '0';
                    
                    // 3D Plot
                    let traces3d = [];
                    
                    if (data.map_points && data.map_points.length > 0) {
                        traces3d.push({
                            x: data.map_points.map(p => p[0]),
                            y: data.map_points.map(p => p[1]),
                            z: data.map_points.map(p => p[2]),
                            mode: 'markers',
                            type: 'scatter3d',
                            marker: { size: 2, color: 'red' },
                            name: 'Map Points'
                        });
                    }
                    
                    if (data.trajectory && data.trajectory.length > 1) {
                        traces3d.push({
                            x: data.trajectory.map(p => p[0]),
                            y: data.trajectory.map(p => p[1]),
                            z: data.trajectory.map(p => p[2]),
                            mode: 'lines+markers',
                            type: 'scatter3d',
                            line: { color: 'blue', width: 4 },
                            marker: { size: 3, color: 'blue' },
                            name: 'Trajectory'
                        });
                    }
                    
                    if (data.current_pose) {
                        traces3d.push({
                            x: [data.current_pose.x],
                            y: [data.current_pose.y],
                            z: [data.current_pose.z],
                            mode: 'markers',
                            type: 'scatter3d',
                            marker: { size: 8, color: 'green' },
                            name: 'Current Pose'
                        });
                    }
                    
                    Plotly.newPlot(plot3dDiv, traces3d, {
                        title: '3D Map & Trajectory',
                        scene: {
                            xaxis: { title: 'X (m)' },
                            yaxis: { title: 'Y (m)' },
                            zaxis: { title: 'Z (m)' }
                        }
                    });
                    
                    // 2D Plot (top view)
                    let traces2d = [];
                    
                    if (data.trajectory && data.trajectory.length > 1) {
                        traces2d.push({
                            x: data.trajectory.map(p => p[0]),
                            y: data.trajectory.map(p => p[1]),
                            mode: 'lines+markers',
                            type: 'scatter',
                            line: { color: 'blue', width: 3 },
                            marker: { size: 4, color: 'blue' },
                            name: 'Path'
                        });
                    }
                    
                    if (data.current_pose) {
                        traces2d.push({
                            x: [data.current_pose.x],
                            y: [data.current_pose.y],
                            mode: 'markers',
                            type: 'scatter',
                            marker: { size: 12, color: 'green' },
                            name: 'Current'
                        });
                    }
                    
                    Plotly.newPlot(plot2dDiv, traces2d, {
                        title: 'Trajectory (Top View)',
                        xaxis: { title: 'X (m)' },
                        yaxis: { title: 'Y (m)', scaleanchor: 'x' }
                    });
                })
                .catch(error => console.error('Error fetching data:', error));
        }
        
        // Update every 2 seconds
        setInterval(updateVisualization, 2000);
        updateVisualization(); // Initial load
    </script>
</body>
</html>'''
        
        with open('/tmp/index.html', 'w') as f:
            f.write(html_content)

    def start_web_server(self):
        """Start HTTP server for web interface"""
        class CustomHandler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory='/tmp', **kwargs)
            
            def do_GET(self):
                if self.path == '/data':
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    
                    try:
                        with open('/tmp/orb_slam_data.json', 'r') as f:
                            self.wfile.write(f.read().encode())
                    except:
                        self.wfile.write(b'{}')
                else:
                    super().do_GET()
        
        try:
            with HTTPServer(('', 8080), CustomHandler) as httpd:
                httpd.serve_forever()
        except Exception as e:
            self.get_logger().error('Web server error: {}'.format(str(e)))

def main(args=None):
    rclpy.init(args=args)
    
    try:
        visualizer = ORBSLAMWebVisualizer()
        
        print("🌐 ORB-SLAM3 Web Visualization Started!")
        print("🔗 Open http://localhost:8080 in your browser")
        print("📊 Real-time visualization available")
        print("🔄 Move your camera to see the map build up")
        print("⏹️  Press Ctrl+C to stop")
        
        rclpy.spin(visualizer)
        
    except KeyboardInterrupt:
        print("\n🛑 Web visualization stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'visualizer' in locals():
            visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
