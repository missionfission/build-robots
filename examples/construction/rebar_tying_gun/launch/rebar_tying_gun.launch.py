#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    """Generate launch description for rebar tying gun system"""
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz visualization'
    )
    
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable camera system'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_camera = LaunchConfiguration('enable_camera')
    
    # Package paths
    pkg_share = FindPackageShare('construction_robots')
    
    # Configuration files
    config_dir = PathJoinSubstitution([pkg_share, 'config'])
    
    # Main controller node
    rebar_controller_node = Node(
        package='construction_robots',
        executable='rebar_tying_controller.py',
        name='rebar_tying_controller',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            PathJoinSubstitution([config_dir, 'rebar_gun_config.yaml'])
        ],
        remappings=[
            ('/rebar_gun/camera/image_raw', '/camera/image_raw'),
            ('/rebar_gun/status', '/robot_status')
        ]
    )
    
    # Hardware interface node
    hardware_interface_node = Node(
        package='construction_robots',
        executable='rebar_gun_hardware.py',
        name='rebar_gun_hardware',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            PathJoinSubstitution([config_dir, 'hardware_config.yaml'])
        ]
    )
    
    # Camera node
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='rebar_camera',
        output='screen',
        parameters=[
            {'video_device': '/dev/video0'},
            {'image_width': 640},
            {'image_height': 480},
            {'pixel_format': 'yuyv'},
            {'camera_frame_id': 'rebar_camera_link'},
            {'io_method': 'mmap'}
        ],
        condition=IfCondition(enable_camera)
    )
    
    # Vision processing node
    vision_node = Node(
        package='construction_robots',
        executable='rebar_vision_processor.py',
        name='rebar_vision_processor',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            PathJoinSubstitution([config_dir, 'vision_config.yaml'])
        ]
    )
    
    # Safety monitor node
    safety_node = Node(
        package='construction_robots',
        executable='safety_monitor.py',
        name='rebar_safety_monitor',
        output='screen',
        parameters=[
            {'robot_type': 'rebar_gun'},
            {'emergency_stop_topic': '/rebar_gun/emergency_stop'},
            {'status_topic': '/rebar_gun/status'}
        ]
    )
    
    # User interface node
    ui_node = Node(
        package='construction_robots',
        executable='rebar_gun_ui.py',
        name='rebar_gun_ui',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # RViz visualization
    rviz_config_file = PathJoinSubstitution([
        pkg_share, 'rviz', 'rebar_gun_visualization.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(enable_rviz),
        output='screen'
    )
    
    # Robot state publisher
    robot_description_file = PathJoinSubstitution([
        pkg_share, 'urdf', 'rebar_gun.urdf.xacro'
    ])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_file},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Transform broadcaster for camera
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_broadcaster',
        arguments=[
            '0.1', '0', '0.05',  # x, y, z
            '0', '0', '0', '1',  # qx, qy, qz, qw
            'rebar_gun_base_link',
            'rebar_camera_link'
        ]
    )
    
    # Diagnostics aggregator
    diagnostics_node = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_dir, 'diagnostics.yaml'])
        ]
    )
    
    # Data logging node
    logger_node = Node(
        package='construction_robots',
        executable='data_logger.py',
        name='rebar_gun_logger',
        output='screen',
        parameters=[
            {'log_directory': '/tmp/rebar_gun_logs'},
            {'log_topics': [
                '/rebar_gun/status',
                '/rebar_gun/tie_count',
                '/camera/image_raw/compressed'
            ]}
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        enable_rviz_arg,
        enable_camera_arg,
        
        # Core nodes
        rebar_controller_node,
        hardware_interface_node,
        vision_node,
        safety_node,
        ui_node,
        
        # Visualization and state
        robot_state_publisher,
        joint_state_publisher,
        camera_tf_node,
        rviz_node,
        
        # Camera system
        camera_node,
        
        # Monitoring and logging
        diagnostics_node,
        logger_node
    ])