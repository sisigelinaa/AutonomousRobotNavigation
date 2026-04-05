import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_bot_controller')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    params_file_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # 1. SLAM Toolbox (Replaces AMCL and Map Server)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file_path]
        ),
        # 2. Lifecycle Manager for SLAM
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['slam_toolbox'] 
            }]
        ),
        # 3. Nav2 Navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': 'True',
                'params_file': params_file_path,
                'autostart': 'True'
            }.items()
        ),

        # 4. Planner controller
        Node(
            package='my_bot_controller',
            executable='planner_controller',
            name='planner_controller',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # 5. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'config', 'nav_config.rviz')],
            parameters=[{'use_sim_time': True}]
        )
    ])