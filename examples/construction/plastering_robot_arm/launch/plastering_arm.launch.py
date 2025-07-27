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
    """Generate launch description for plastering robot arm system"""
    
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
    
    enable_moveit_arg = DeclareLaunchArgument(
        'enable_moveit',
        default_value='true',
        description='Enable MoveIt motion planning'
    )
    
    enable_teleop_arg = DeclareLaunchArgument(
        'enable_teleop',
        default_value='true',
        description='Enable teleoperation interface'
    )
    
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.100',
        description='IP address of the robot controller'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_moveit = LaunchConfiguration('enable_moveit')
    enable_teleop = LaunchConfiguration('enable_teleop')
    robot_ip = LaunchConfiguration('robot_ip')
    
    # Package paths
    pkg_share = FindPackageShare('construction_robots')
    moveit_config_pkg = FindPackageShare('plastering_arm_moveit_config')
    
    # Configuration files
    config_dir = PathJoinSubstitution([pkg_share, 'config'])
    
    # Main controller node
    plastering_controller_node = Node(
        package='construction_robots',
        executable='plastering_arm_controller.py',
        name='plastering_arm_controller',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_ip': robot_ip},
            PathJoinSubstitution([config_dir, 'plastering_arm_config.yaml'])
        ]
    )
    
    # Robot hardware interface
    robot_driver_node = Node(
        package='ur_robot_driver',
        executable='ur_ros2_control_node',
        name='ur_ros2_control_node',
        output='screen',
        parameters=[
            {'robot_ip': robot_ip},
            {'script_filename': '/tmp/ros_control.urscript'},
            {'output_recipe_filename': '/tmp/rtde_output_recipe.txt'},
            {'input_recipe_filename': '/tmp/rtde_input_recipe.txt'},
            PathJoinSubstitution([config_dir, 'ur_controllers.yaml'])
        ]
    )
    
    # Tool system interface
    tool_interface_node = Node(
        package='construction_robots',
        executable='plastering_tools_interface.py',
        name='plastering_tools_interface',
        output='screen',
        parameters=[
            {'spray_system_port': '/dev/ttyUSB0'},
            {'mixer_system_port': '/dev/ttyUSB1'},
            PathJoinSubstitution([config_dir, 'tool_config.yaml'])
        ]
    )
    
    # Surface scanning node
    surface_scanner_node = Node(
        package='construction_robots',
        executable='surface_scanner.py',
        name='surface_scanner',
        output='screen',
        parameters=[
            {'scanner_type': 'structured_light'},
            {'scan_resolution': 0.001},  # 1mm resolution
            PathJoinSubstitution([config_dir, 'scanner_config.yaml'])
        ]
    )
    
    # Force/torque sensor node
    ft_sensor_node = Node(
        package='construction_robots',
        executable='ft_sensor_driver.py',
        name='ft_sensor',
        output='screen',
        parameters=[
            {'sensor_type': 'ati_gamma'},
            {'sensor_port': '/dev/ttyUSB2'},
            {'publish_rate': 100.0}  # 100 Hz
        ]
    )
    
    # Teleoperation interface
    teleop_interface_node = Node(
        package='construction_robots',
        executable='plastering_teleop_interface.py',
        name='plastering_teleop_interface',
        output='screen',
        parameters=[
            {'interface_type': 'joystick'},  # joystick, vr, tablet
            {'force_feedback_enabled': True},
            {'safety_limits_enabled': True}
        ],
        condition=IfCondition(enable_teleop)
    )
    
    # Safety monitor
    safety_monitor_node = Node(
        package='construction_robots',
        executable='safety_monitor.py',
        name='plastering_safety_monitor',
        output='screen',
        parameters=[
            {'robot_type': 'plastering_arm'},
            {'max_force_limit': 50.0},  # Newtons
            {'workspace_limits': [
                {'x_min': -1.0, 'x_max': 1.0},
                {'y_min': -1.0, 'y_max': 1.0},
                {'z_min': 0.0, 'z_max': 2.0}
            ]}
        ]
    )
    
    # Quality control system
    quality_control_node = Node(
        package='construction_robots',
        executable='plastering_quality_control.py',
        name='quality_control',
        output='screen',
        parameters=[
            {'thickness_tolerance': 2.0},  # mm
            {'surface_roughness_limit': 0.5},
            {'inspection_frequency': 10}  # every 10 points
        ]
    )
    
    # MoveIt configuration
    moveit_config = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                moveit_config_pkg,
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'publish_monitored_planning_scene': 'true'
        }.items(),
        condition=IfCondition(enable_moveit)
    )
    
    # RViz with MoveIt configuration
    rviz_config_file = PathJoinSubstitution([
        moveit_config_pkg, 'config', 'moveit.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(enable_rviz),
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Robot state publisher
    robot_description_file = PathJoinSubstitution([
        pkg_share, 'urdf', 'plastering_arm.urdf.xacro'
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
    
    # Joint state publisher GUI (for manual testing)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_sim_time)  # Only in simulation
    )
    
    # Camera system for surface monitoring
    surface_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='surface_camera',
        output='screen',
        parameters=[
            {'camera_name': 'surface_camera'},
            {'color_width': 1280},
            {'color_height': 720},
            {'depth_width': 1280},
            {'depth_height': 720},
            {'enable_pointcloud': True},
            {'pointcloud_texture_stream': 'RS2_STREAM_COLOR'}
        ]
    )
    
    # Transform publishers for sensors
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='surface_camera_tf',
        arguments=[
            '0.5', '0', '0.3',  # x, y, z
            '0', '0.707', '0', '0.707',  # qx, qy, qz, qw (90 deg rotation)
            'base_link',
            'surface_camera_link'
        ]
    )
    
    ft_sensor_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ft_sensor_tf',
        arguments=[
            '0', '0', '0',  # x, y, z
            '0', '0', '0', '1',  # qx, qy, qz, qw
            'tool0',
            'ft_sensor_link'
        ]
    )
    
    # Material management system
    material_manager_node = Node(
        package='construction_robots',
        executable='material_manager.py',
        name='material_manager',
        output='screen',
        parameters=[
            {'tank_capacity': 50.0},  # Liters
            {'mixing_time': 120.0},  # Seconds
            {'material_types': ['cement_plaster', 'gypsum_plaster', 'lime_plaster']}
        ]
    )
    
    # Data logging and analysis
    data_logger_node = Node(
        package='construction_robots',
        executable='data_logger.py',
        name='plastering_data_logger',
        output='screen',
        parameters=[
            {'log_directory': '/tmp/plastering_logs'},
            {'log_topics': [
                '/plastering/status',
                '/joint_states',
                '/wrench',
                '/surface_scan',
                '/quality_metrics'
            ]},
            {'log_frequency': 10.0}  # Hz
        ]
    )
    
    # Performance monitor
    performance_monitor_node = Node(
        package='construction_robots',
        executable='performance_monitor.py',
        name='performance_monitor',
        output='screen',
        parameters=[
            {'coverage_tracking': True},
            {'efficiency_metrics': True},
            {'quality_tracking': True}
        ]
    )
    
    # Web interface for remote monitoring
    web_interface_node = Node(
        package='construction_robots',
        executable='web_interface.py',
        name='web_interface',
        output='screen',
        parameters=[
            {'port': 8080},
            {'enable_video_stream': True},
            {'enable_remote_control': False}  # Safety: disable remote control
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        enable_rviz_arg,
        enable_moveit_arg,
        enable_teleop_arg,
        robot_ip_arg,
        
        # Core robot system
        plastering_controller_node,
        robot_driver_node,
        robot_state_publisher,
        joint_state_publisher_gui,
        
        # MoveIt motion planning
        moveit_config,
        
        # Tool and sensor systems
        tool_interface_node,
        surface_scanner_node,
        ft_sensor_node,
        surface_camera_node,
        
        # Teleoperation and safety
        teleop_interface_node,
        safety_monitor_node,
        
        # Quality and material management
        quality_control_node,
        material_manager_node,
        
        # Visualization and monitoring
        rviz_node,
        data_logger_node,
        performance_monitor_node,
        web_interface_node,
        
        # Transform publishers
        camera_tf_node,
        ft_sensor_tf_node
    ])