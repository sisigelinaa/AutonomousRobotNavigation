import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share       = get_package_share_directory('my_bot_controller')
    nav_config      = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    scenario_config = os.path.join(pkg_share, 'config', 'scenario_goals.yaml')
    map_file        = os.path.join(pkg_share, 'maps', 't3_house.yaml')
    rviz_config     = os.path.join(pkg_share, 'config', 'nav_config.rviz')

    return LaunchDescription([
        # 1. Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'use_sim_time': True, 'yaml_filename': map_file}]
        ),
        # 2. AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[nav_config]
        ),
        # 3. Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[{'use_sim_time': True,
                         'autostart': True,
                         'node_names': ['map_server', 'amcl']}]
        ),
        # 4. Planner Controller
        Node(
            package='my_bot_controller',
            executable='planner_controller',
            output='screen',
            parameters=[{'use_sim_time': True, 'scan_topic': '/scan'}]
        ),
        # 5. Scenario Runner — identical goals to radar run
        # Node(
        #     package='my_bot_controller',
        #     executable='scenario_runner',
        #     name='scenario_runner',
        #     output='screen',
        #     parameters=[scenario_config]
        # ),
        # # 6. Metrics Logger — tagged as lidar
        # Node(
        #     package='my_bot_controller',
        #     executable='metrics_logger',
        #     name='metrics_logger',
        #     output='screen',
        #     parameters=[{'sensor_type': 'lidar'}]
        # ),
        # 7. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config]
        ),
    ])
