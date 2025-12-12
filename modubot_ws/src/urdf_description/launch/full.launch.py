#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    pkg_robot = get_package_share_directory('urdf_description')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # ---------- Launch arguments ----------
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use /clock from Gazebo'
    )

    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_robot, 'maps', 'PisoInferior.yaml')
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_robot, 'config', 'nav2_params.yaml')
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, 'launch', 'gazebo.launch.py')
        )
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'false',
            'slam': 'False',
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'True',
            'use_respawn': 'False',
        }.items()
    )

    rviz_config = os.path.join(
        pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz'
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ---------- LaunchDescription ----------
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map_yaml)
    ld.add_action(declare_params_file)
    ld.add_action(declare_use_rviz)

    ld.add_action(gazebo_launch)
    ld.add_action(nav2_bringup)
    ld.add_action(rviz)

    return ld
