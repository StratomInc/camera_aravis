# Copyright (c) 2020, Stratom Inc.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('camera_aravis'),
        'config',
        'lucid.yaml'
        )

    # Lucid Camera Node
    lucid_cam = Node(
        package='camera_aravis',
        node_executable='camnode',
        node_name='lucid_camera',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        lucid_cam
    ])

