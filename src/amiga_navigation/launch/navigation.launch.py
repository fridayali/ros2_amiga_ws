import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('amiga_navigation')

    namespace     = LaunchConfiguration('namespace')
    use_sim_time  = LaunchConfiguration('use_sim_time')
    autostart     = LaunchConfiguration('autostart')
    params_file   = LaunchConfiguration('params_file')
    log_level     = LaunchConfiguration('log_level')
    container_name = LaunchConfiguration('container_name')

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

    # Nav2'nin resmi composition tavsiyesi: tüm server'ları ayrı process
    # değil, TEK component container içinde composable node olarak çalıştır.
    # Bu, process-level context-switch yükünü ve ayrı DDS participant
    # sayısını azaltır — Jetson gibi CPU kısıtlı donanımda kazanç
    # (https://docs.nav2.org/tuning/index.html). use_intra_process_comms
    # KASITLI OLARAK kapalı: bazı node'lar (örn. smoother_server,
    # costmap'lerin /map static_layer aboneliği) transient_local
    # durability kullanıyor, ROS2'nin intra-process comms'i SADECE
    # volatile durability'yi destekliyor — açarsak "intraprocess
    # communication allowed only with volatile durability" hatasıyla
    # lifecycle bringup tamamen patlıyor. respawn/use_respawn artık yok:
    # tek bir component crash olursa tüm container etkilenir, bu
    # composition'ın bilinen tradeoff'u — container'ın kendisi
    # respawn=true ile yeniden başlar.
    composable_nodes = [
        ComposableNode(
            package='nav2_controller', plugin='nav2_controller::ControllerServer',
            name='controller_server',
            parameters=[configured_params],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        ComposableNode(
            package='nav2_smoother', plugin='nav2_smoother::SmootherServer',
            name='smoother_server',
            parameters=[configured_params],
            remappings=remappings,
        ),
        ComposableNode(
            package='nav2_planner', plugin='nav2_planner::PlannerServer',
            name='planner_server',
            parameters=[configured_params],
            remappings=remappings,
        ),
        ComposableNode(
            package='nav2_behaviors', plugin='behavior_server::BehaviorServer',
            name='behavior_server',
            parameters=[configured_params],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        ComposableNode(
            package='nav2_bt_navigator', plugin='nav2_bt_navigator::BtNavigator',
            name='bt_navigator',
            parameters=[configured_params],
            remappings=remappings,
        ),
        ComposableNode(
            package='nav2_waypoint_follower', plugin='nav2_waypoint_follower::WaypointFollower',
            name='waypoint_follower',
            parameters=[configured_params],
            remappings=remappings,
        ),
        ComposableNode(
            package='nav2_velocity_smoother', plugin='nav2_velocity_smoother::VelocitySmoother',
            name='velocity_smoother',
            parameters=[configured_params],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        ComposableNode(
            package='nav2_collision_monitor', plugin='nav2_collision_monitor::CollisionMonitor',
            name='collision_monitor',
            parameters=[configured_params],
            remappings=remappings,
        ),
        ComposableNode(
            package='nav2_lifecycle_manager', plugin='nav2_lifecycle_manager::LifecycleManager',
            name='lifecycle_manager_navigation',
            parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
        ),
    ]

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument('namespace',    default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart',    default_value='true'),
        DeclareLaunchArgument('params_file',
            default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('log_level',    default_value='info'),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),

        GroupAction(actions=[
            SetParameter('use_sim_time', use_sim_time),

            ComposableNodeContainer(
                name=container_name,
                namespace=namespace,
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=composable_nodes,
                arguments=['--ros-args', '--log-level', log_level],
                output='screen',
                respawn=True,
                respawn_delay=2.0,
            ),
        ]),
    ])
