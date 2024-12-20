#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Set the path to different files and folders
    pkg_name = 'urban_farming_robots'
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)
    
    # Paths to files and folders
    default_rviz_config_path = os.path.join(pkg_share, 'config/planting_robot.rviz')
    default_urdf_model_path = os.path.join(pkg_share, 'urdf/planting_robot.urdf')
    robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml')
    world_path = os.path.join(pkg_share, 'worlds/greenhouse.world')
    
    # Launch configuration variables
    urdf_model = LaunchConfiguration('urdf_model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # Declare launch arguments
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model', 
        default_value=default_urdf_model_path, 
        description='Absolute path to robot urdf file')
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
        
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RViz')
        
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RViz config file')

    # Start Gazebo with greenhouse world
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so'],
        output='screen')

    # Publish robot state
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_model])
        }])

    # Spawn the robot in Gazebo
    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'planting_robot',
            '-topic', 'robot_description'
        ],
        output='screen')

    # Start RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz))

    # Start Controllers
    start_controller_cmd = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 'arm_controller'])

    # Start Robot Localization using EKF
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path])

    # Create and return launch description
    ld = LaunchDescription()

    # Declare launch options
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add nodes to launch description
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_controller_cmd)
    ld.add_action(start_robot_localization_cmd)

    return ld 