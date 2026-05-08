import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_lidar = get_package_share_directory('amiga_lidar')

    serial_port     = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id        = LaunchConfiguration('frame_id')
    scan_mode       = LaunchConfiguration('scan_mode')
    use_filter      = LaunchConfiguration('use_filter')

    laser_filter_config = os.path.join(pkg_lidar, 'config', 'laser_filter.yaml')

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lidar, 'launch', 'rplidar.launch.py')
        ),
        launch_arguments={
            'serial_port':     serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id':        frame_id,
            'scan_mode':       scan_mode,
        }.items(),
    )

    # laser_filters scan_to_scan_filter_chain:
    #   /scan  →  filtreleme  →  /scan_filtered
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        output='screen',
        parameters=[laser_filter_config],
        remappings=[
            ('scan',          '/scan'),
            ('scan_filtered', '/scan_filtered'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port',     default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('frame_id',        default_value='laser_link'),
        DeclareLaunchArgument('scan_mode',       default_value='Sensitivity'),
        DeclareLaunchArgument('use_filter',      default_value='true',
                              description='Laser filter düğümünü başlat'),

        rplidar_launch,
        laser_filter_node,
    ])
