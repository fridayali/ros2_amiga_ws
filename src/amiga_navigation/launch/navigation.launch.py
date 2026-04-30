import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('amiga_navigation')

    namespace     = LaunchConfiguration('namespace')
    use_sim_time  = LaunchConfiguration('use_sim_time')
    autostart     = LaunchConfiguration('autostart')
    params_file   = LaunchConfiguration('params_file')
    use_respawn   = LaunchConfiguration('use_respawn')
    log_level     = LaunchConfiguration('log_level')

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
    ]

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={'autostart': autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument('namespace',    default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart',    default_value='true'),
        DeclareLaunchArgument('params_file',
            default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_respawn',  default_value='true'),
        DeclareLaunchArgument('log_level',    default_value='info'),

        GroupAction(actions=[
            SetParameter('use_sim_time', use_sim_time),

            Node(package='nav2_controller',   executable='controller_server',
                 output='screen', respawn=use_respawn, respawn_delay=2.0,
                 parameters=[configured_params],
                 arguments=['--ros-args', '--log-level', log_level],
                 remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),

            Node(package='nav2_smoother',     executable='smoother_server',
                 name='smoother_server', output='screen',
                 respawn=use_respawn, respawn_delay=2.0,
                 parameters=[configured_params],
                 arguments=['--ros-args', '--log-level', log_level],
                 remappings=remappings),

            Node(package='nav2_planner',      executable='planner_server',
                 name='planner_server', output='screen',
                 respawn=use_respawn, respawn_delay=2.0,
                 parameters=[configured_params],
                 arguments=['--ros-args', '--log-level', log_level],
                 remappings=remappings),

            Node(package='nav2_behaviors',    executable='behavior_server',
                 name='behavior_server', output='screen',
                 respawn=use_respawn, respawn_delay=2.0,
                 parameters=[configured_params],
                 arguments=['--ros-args', '--log-level', log_level],
                 remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),

            Node(package='nav2_bt_navigator', executable='bt_navigator',
                 name='bt_navigator', output='screen',
                 respawn=use_respawn, respawn_delay=2.0,
                 parameters=[configured_params],
                 arguments=['--ros-args', '--log-level', log_level],
                 remappings=remappings),

            Node(package='nav2_waypoint_follower', executable='waypoint_follower',
                 name='waypoint_follower', output='screen',
                 respawn=use_respawn, respawn_delay=2.0,
                 parameters=[configured_params],
                 arguments=['--ros-args', '--log-level', log_level],
                 remappings=remappings),

            Node(package='nav2_velocity_smoother', executable='velocity_smoother',
                 name='velocity_smoother', output='screen',
                 respawn=use_respawn, respawn_delay=2.0,
                 parameters=[configured_params],
                 arguments=['--ros-args', '--log-level', log_level],
                 remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),

            Node(package='nav2_collision_monitor', executable='collision_monitor',
                 name='collision_monitor', output='screen',
                 respawn=use_respawn, respawn_delay=2.0,
                 parameters=[configured_params],
                 arguments=['--ros-args', '--log-level', log_level],
                 remappings=remappings),

            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 arguments=['--ros-args', '--log-level', log_level],
                 parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}]),
        ]),
    ])
