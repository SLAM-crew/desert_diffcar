import os 

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


rviz_config = '/workspaces/dev_humble/workspace/src/mini_nav2/rviz/default.rviz'

def generate_launch_description():

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ]),
            launch_arguments={
                'params_file': '/workspaces/dev_humble/workspace/src/mini_nav2/params/nav2_params.yaml',
                'use_localization': 'False',
                'use_sim_time': 'True',
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ]),
            launch_arguments={
                'slam_params_file': '/workspaces/dev_humble/workspace/src/mini_nav2/config/mapper_params_online_async.yaml'
            }.items()
        ),        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('survey'),
                    'launch',
                    'ozyland.launch.py'
                ])
            ])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('explore_lite'),
                    'launch',
                    'explore.launch.py'
                ])
            ])
        ),
        Node(
            package    = "mini_nav2",
            executable = "qr_detect.py",
            name = 'qr_scan3',
            output     = "screen",
            #emulate_tty=True,
            ), 
        Node(
            package    = "tf2_ros",
            executable = "static_transform_publisher",
            name = 'base_link_to_base_footstep_static_publisher',
            arguments  = ["0", "0", "0.775", "0", "0", "0", "base_link", "base_footprint"],
            output     = "screen"),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
      )
    ])