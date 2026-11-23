from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='41451'),
        DeclareLaunchArgument('vehicle_name', default_value='PX4SITL'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
    ]
    return LaunchDescription(arguments + [
        Node(
            package='airsim_ros2',
            executable='airsim_bridge',
            name='airsim_bridge',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'vehicle_name': LaunchConfiguration('vehicle_name'),
                'publish_tf': LaunchConfiguration('publish_tf'),
            }],
        ),
    ])
