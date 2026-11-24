"""Read sensors from an already running simulator; does not start or control it."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('airsim_ros2'), 'config', 'sensor_bridge.yaml'])),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='41451'),
        DeclareLaunchArgument('vehicle_name', default_value='UAV_A'),
        DeclareLaunchArgument('namespace', default_value='uav_a'),
        DeclareLaunchArgument('frame_prefix', default_value='uav_a'),
        DeclareLaunchArgument('publish_clock', default_value='true'),
    ]
    parameters = {name: LaunchConfiguration(name)
                  for name in ('host', 'vehicle_name', 'frame_prefix')}
    parameters['port'] = ParameterValue(LaunchConfiguration('port'), value_type=int)
    parameters['publish_clock'] = ParameterValue(LaunchConfiguration('publish_clock'), value_type=bool)
    return LaunchDescription(arguments + [Node(
        package='airsim_ros2', executable='airsim_sensor_bridge', name='airsim_sensor_bridge',
        namespace=LaunchConfiguration('namespace'), output='screen',
        parameters=[LaunchConfiguration('params_file'), parameters],
    )])
