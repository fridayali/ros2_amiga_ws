from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port      = LaunchConfiguration('port')
    address   = LaunchConfiguration('address')

    return LaunchDescription([
        DeclareLaunchArgument('port',    default_value='9090',
                              description='WebSocket port for rosbridge'),
        DeclareLaunchArgument('address', default_value='',
                              description='Bind address (empty = all interfaces)'),

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': port,
                'address': address,
                'use_compression': False,
                'authenticate': False,
            }],
        ),

        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
            output='screen',
        ),
    ])
