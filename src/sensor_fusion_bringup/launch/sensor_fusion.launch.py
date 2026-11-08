"""Start the EKF using the module YAML and an optional clock override."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    config = LaunchConfiguration('config_file').perform(context)
    clock = LaunchConfiguration('use_sim_time').perform(context)
    overrides = {} if clock == 'config' else {'use_sim_time': clock == 'true'}
    return [Node(package='ekf', executable='ekf_node', name='ekf',
                 parameters=[config, overrides], output='screen')]


def generate_launch_description():
    default = str(Path(get_package_share_directory('sensor_fusion_bringup')) / 'config/ekf.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default),
        DeclareLaunchArgument('use_sim_time', default_value='config', choices=['config', 'true', 'false']),
        OpaqueFunction(function=start),
    ])
