"""Launch the hardware tracker, inactive until its start service is called."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    """Keep the ROBOTIS bringup separate; never open the serial port here."""
    config = os.path.join(get_package_share_directory('trajectory_controller'),
                          'config', 'burger.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('trajectory', default_value='circle',
                              choices=['circle', 'eight']),
        DeclareLaunchArgument('stamped_cmd_vel', default_value='false',
                              choices=['true', 'false']),
        Node(package='trajectory_controller', executable='command_guard',
             name='command_guard', output='screen',
             parameters=[LaunchConfiguration('params_file'),
                         {'use_sim_time': False, 'stamped_cmd_vel': ParameterValue(
                             LaunchConfiguration('stamped_cmd_vel'), value_type=bool)}]),
        Node(package='trajectory_controller', executable='tracking_node',
             name='tracking_node', output='screen',
             parameters=[LaunchConfiguration('params_file'),
                         {'trajectory': LaunchConfiguration('trajectory'),
                          'use_sim_time': False}]),
    ])
