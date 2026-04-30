import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_slam    = get_package_share_directory('slam_toolbox')
    pkg_amiga   = get_package_share_directory('amiga_slam')
    params_file = os.path.join(pkg_amiga, 'config', 'mapper_params_online_async.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time':  use_sim_time,
                'params_file':   params_file,
            }.items(),
        ),
    ])
