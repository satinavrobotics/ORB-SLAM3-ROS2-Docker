#!/usr/bin/env python3
"""
Launch file for SATI ROS Camera Stack with integrated image converter
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='/base/axi/pcie@1000120000/rp1/i2c@88000/ov9281@60',
        description='Camera device path'
    )
    
    format_arg = DeclareLaunchArgument(
        'format',
        default_value='R8',
        description='Camera format (R8, YUYV, etc.)'
    )
    
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='640',
        description='Camera width'
    )
    
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='480',
        description='Camera height'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Camera FPS'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera',
        description='Camera frame ID'
    )
    
    enable_converter_arg = DeclareLaunchArgument(
        'enable_converter',
        default_value='true',
        description='Enable image converter for mono8 output'
    )
    
    # Include the camera_ros launch file
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('camera_ros'),
                'launch',
                'camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera': LaunchConfiguration('camera'),
            'format': LaunchConfiguration('format'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': LaunchConfiguration('fps'),
            'frame_id': LaunchConfiguration('frame_id'),
        }.items()
    )
    
    # Image converter node
    image_converter_node = Node(
        package='sati_ros_camera_stack',
        executable='image_converter_node.py',
        name='sati_image_converter',
        parameters=[{
            'input_topic': '/camera/image_raw',
            'output_topic': '/camera/image_mono8'
        }],
        condition=LaunchConfiguration('enable_converter')
    )
    
    return LaunchDescription([
        camera_arg,
        format_arg,
        width_arg,
        height_arg,
        fps_arg,
        frame_id_arg,
        enable_converter_arg,
        camera_launch,
        image_converter_node,
    ])
