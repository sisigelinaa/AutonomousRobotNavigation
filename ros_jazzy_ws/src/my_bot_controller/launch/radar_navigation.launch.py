import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_my_bot = get_package_share_directory('my_bot_controller')
    nav_config      = os.path.join(pkg_my_bot, 'config', 'radar_nav2_params.yaml')
    scenario_config = os.path.join(pkg_my_bot, 'config', 'scenario_goals.yaml')
    rviz_config     = os.path.join(pkg_my_bot, 'config', 'nav_config.rviz')
    #CHANGE THIS FILE NAME TO CHANGE MAP
    map_file        = os.path.join(pkg_my_bot, 'maps', 't3_house.yaml')

    node_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True, 'yaml_filename': map_file}]
    )

    node_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[nav_config]
    )

    node_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{'use_sim_time': True,
                     'autostart': True,
                     'node_names': ['map_server', 'amcl']}]
    )

    node_planner_controller = Node(
        package='my_bot_controller',
        executable='planner_controller',
        name='planner_controller',
        output='screen',
        parameters=[{'use_sim_time': True, 'scan_topic': '/radar_scan'}]
    )

    node_scenario_runner = Node(
        package='my_bot_controller',
        executable='scenario_runner',
        name='scenario_runner',
        output='screen',
        parameters=[scenario_config]
    )

    node_metrics_logger = Node(
        package='my_bot_controller',
        executable='metrics_logger',
        name='metrics_logger',
        output='screen',
        parameters=[{'sensor_type': 'radar'}]
    )

    node_breathing_detector = Node(
        package='my_bot_controller',
        executable='breathing_detector',
        name='breathing_detector',
        output='screen',
        parameters=[{'scan_topic': '/radar_scan', 'scan_rate': 10.0}]
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        node_map_server,
        node_amcl,
        node_lifecycle,
        node_planner_controller,
        # node_scenario_runner,
        # node_metrics_logger,
        node_breathing_detector,
        node_rviz,
    ])
