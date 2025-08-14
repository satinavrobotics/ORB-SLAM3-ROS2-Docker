#!/usr/bin/env python3
"""
Launch file for SATI ROS Camera Stack with ORB-SLAM3 integration
Includes camera, image converter, and ORB-SLAM3 monocular SLAM
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments for camera
    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='/base/axi/pcie@1000120000/rp1/i2c@80000/ov9281@60',
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
        default_value='400',
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

    role_arg = DeclareLaunchArgument(
        'role',
        default_value='video',
        description='Camera role (video, still, raw)'
    )

    exposure_time_arg = DeclareLaunchArgument(
        'exposure_time',
        default_value='8333',
        description='Camera exposure time in microseconds'
    )
    
    # Declare launch arguments for ORB-SLAM3
    vocabulary_path_arg = DeclareLaunchArgument(
        'vocabulary_path',
        default_value='/home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt',
        description='Path to ORB-SLAM3 vocabulary file'
    )
    
    settings_path_arg = DeclareLaunchArgument(
        'settings_path',
        default_value='/root/colcon_ws/src/orb_slam3_ros2_wrapper/params/arducam_mono.yaml',
        description='Path to ORB-SLAM3 settings file'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic_name',
        default_value='/camera/image_mono8',
        description='Image topic for ORB-SLAM3'
    )
    
    visualization_arg = DeclareLaunchArgument(
        'visualization',
        default_value='false',
        description='Enable ORB-SLAM3 visualization'
    )
    
    enable_converter_arg = DeclareLaunchArgument(
        'enable_converter',
        default_value='true',
        description='Enable image converter for mono8 output'
    )

    # Frame configuration arguments for ORB-SLAM3
    robot_base_frame_arg = DeclareLaunchArgument(
        'robot_base_frame',
        default_value='base_footprint',
        description='Robot base frame ID for ORB-SLAM3'
    )

    global_frame_arg = DeclareLaunchArgument(
        'global_frame',
        default_value='orb_map',
        description='Global frame ID for ORB-SLAM3 (usually map)'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame ID for ORB-SLAM3'
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Enable TF publishing from ORB-SLAM3'
    )

    # Camera transform arguments
    camera_x_arg = DeclareLaunchArgument(
        'camera_x',
        default_value='0.16',
        description='Camera X offset from base_footprint (meters)'
    )

    camera_y_arg = DeclareLaunchArgument(
        'camera_y',
        default_value='0.0',
        description='Camera Y offset from base_footprint (meters)'
    )

    camera_z_arg = DeclareLaunchArgument(
        'camera_z',
        default_value='0.0',
        description='Camera Z offset from base_footprint (meters)'
    )

    camera_roll_arg = DeclareLaunchArgument(
        'camera_roll',
        default_value='0.0',
        description='Camera roll rotation (radians)'
    )

    camera_pitch_arg = DeclareLaunchArgument(
        'camera_pitch',
        default_value='0.0',
        description='Camera pitch rotation (radians)'
    )

    camera_yaw_arg = DeclareLaunchArgument(
        'camera_yaw',
        default_value='0.0',
        description='Camera yaw rotation (radians)'
    )

    # Camera node (direct execution)
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
        parameters=[{
            'format': LaunchConfiguration('format'),
            'role': LaunchConfiguration('role'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': LaunchConfiguration('fps'),
            'exposure_time': LaunchConfiguration('exposure_time'),
            'camera': LaunchConfiguration('camera'),
            'frame_id': LaunchConfiguration('frame_id')
        }],
        output='screen'
    )
    
    # Image converter node
    image_converter_node = Node(
        package='sati_ros_camera_stack',
        executable='image_converter_node',
        name='sati_image_converter',
        parameters=[{
            'input_topic': '/camera/image_raw',
            'output_topic': LaunchConfiguration('image_topic_name')
        }],
        condition=IfCondition(LaunchConfiguration('enable_converter')),
        output='screen'
    )
    
    # Static transform publisher for camera
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=[
            LaunchConfiguration('camera_x'),
            LaunchConfiguration('camera_y'),
            LaunchConfiguration('camera_z'),
            LaunchConfiguration('camera_yaw'),
            LaunchConfiguration('camera_pitch'),
            LaunchConfiguration('camera_roll'),
            LaunchConfiguration('robot_base_frame'),
            LaunchConfiguration('frame_id')
        ],
        output='screen'
    )

    # ORB-SLAM3 monocular node
    orbslam3_node = Node(
        package='orb_slam3_ros2_wrapper',
        executable='mono',
        name='orb_slam3_mono',
        arguments=[
            LaunchConfiguration('vocabulary_path'),
            LaunchConfiguration('settings_path')
        ],
        parameters=[{
            'image_topic_name': LaunchConfiguration('image_topic_name'),
            'visualization': LaunchConfiguration('visualization'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'global_frame': LaunchConfiguration('global_frame'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'publish_tf': LaunchConfiguration('publish_tf')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        # Camera arguments
        camera_arg,
        format_arg,
        width_arg,
        height_arg,
        fps_arg,
        frame_id_arg,
        role_arg,
        exposure_time_arg,
        # ORB-SLAM3 arguments
        vocabulary_path_arg,
        settings_path_arg,
        image_topic_arg,
        visualization_arg,
        enable_converter_arg,
        # Frame configuration arguments
        robot_base_frame_arg,
        global_frame_arg,
        odom_frame_arg,
        publish_tf_arg,
        # Camera transform arguments
        camera_x_arg,
        camera_y_arg,
        camera_z_arg,
        camera_roll_arg,
        camera_pitch_arg,
        camera_yaw_arg,
        # Nodes
        camera_node,
        image_converter_node,
        static_tf_node,
        orbslam3_node,
    ])
