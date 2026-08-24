#!/usr/bin/env python3
# File   : comm_relay.launch.py
# Purpose: Start the comm relay node with the shared YAML parameters.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("scout2map_comm")
    default_params = os.path.join(pkg_share, "config", "comm_relay.yaml")

    params_arg = DeclareLaunchArgument(
        "params_file", default_value=default_params,
        description="YAML file with comm_relay parameters")

    relay = Node(
        package="scout2map_comm",
        executable="comm_relay_node",
        name="comm_relay",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_arg, relay])
