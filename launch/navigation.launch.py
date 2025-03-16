from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('prometheus_core')
    
    # Define the path to the parameter file
    params_file_path = os.path.join(pkg_dir, 'config', 'navigation_params.yaml')
    
    # Declare the parameter file argument with default value
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file_path,
        description='Path to the ROS2 parameters file to use'
    )
    
    # Create the node
    navigation_node = Node(
        package='prometheus_core',
        executable='prometheus_navigation_node',
        name='prometheus_navigation_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        emulate_tty=True
    )
    
    # Return the launch description
    return LaunchDescription([
        params_file_arg,
        navigation_node
    ]) 