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
        'manta.yaml'
        )

    # Manta Camera Node
    manta_cam = Node(
        package='camera_aravis',
        executable='camnode',
        name='manta_camera',
        output='screen',
        parameters=[config]
    )

    # Image Proc Node
    image_proc = Node(
        package='image_proc',
        executable='image_proc',
        name='manta_image_proc',
        namespace='manta_camera',
        output='screen'
    )

    return LaunchDescription([
        manta_cam,
        image_proc
    ])

