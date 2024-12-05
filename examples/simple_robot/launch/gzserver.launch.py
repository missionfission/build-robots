from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Launch arguments
    world = LaunchConfiguration('world')
    
    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value='',
        description='Full path to world file to load')
        
    # Start Gazebo server
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', world],
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(declare_world_cmd)
    ld.add_action(start_gazebo_server_cmd)
    
    return ld 