import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    
    package_name = 'galax_navigation'
    
    nav2_config = os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory(package_name), 'config', 'nav2_default_view.rviz')
    map_file = os.path.join(get_package_share_directory(package_name), 'map', 'my_map4.yaml')
    bt_xml_path = os.path.join(get_package_share_directory(package_name), 'config', 'navigate_w_replanning_and_recovery.xml')
    
    return LaunchDescription([
        
        # Map Server Node - Provides the map to other nodes
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_config, {'yaml_filename':map_file}, {'topic_name': '/map'}]),
            
        # AMCL Node - Handles localization using particle filter
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_config],
            arguments=['--ros-args', '--log-level', 'warn']),
        
        # Controller Server - Executes the path following behavior
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_config]),

        # Planner Server - Generates global path plans
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_config]),
            
        # Recovery Server - Handles recovery behaviors when robot gets stuck
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen'),

        # Behavior Tree Navigator - Manages navigation tasks using behavior trees
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_config, {'bt_xml_filename': bt_xml_path}]),
        
        # Lifecycle Manager - Manages the lifecycle states of the navigation nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'controller_server',
                                        'planner_server',
                                        'behavior_server',
                                        'bt_navigator']}]),
        
        # RViz2 - Visualization tool for monitoring navigation
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            output='screen'),
        
        # Robot bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('galax_bringup'),
                'launch',
                'launch_real_robot.launch.py'
            )])),
        
    ])
    
