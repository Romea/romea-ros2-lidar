# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from romea_imu_meta_bringup.meta_description import generate_urdf_description


def launch_setup(context, *args, **kwargs):

    simulator_type = LaunchConfiguration("simulator").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    meta_description_file_path = LaunchConfiguration("meta_description_file_path").perform(context)


    simulation = LaunchDescription()

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_simulation_meta_bringup")
            + "/launch/simulator.launch.py"
        ),
        launch_arguments={'simulator_type': simulator_type}.items(),
    )

    simulation.add_action(simulator)

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_simulation_meta_bringup")
            + "/launch/entity.launch.py"
        ),
        launch_arguments={
            'simulator_type': simulator_type,
            'entity_type': "lidar",
            'robot_namespace': robot_namespace,
            'meta_description_file_path': meta_description_file_path,
        }.items(),
    )

    simulation.add_action(lidar)

    nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_lidar_meta_bringup")
            + "/launch/lidar.launch.py"
        ),
        launch_arguments={
            'robot_namespace': robot_namespace,
            'mode': f"simulation_{simulator_type}",
            'meta_description_file_path': meta_description_file_path,
        }.items(),
    )

    simulation.add_action(nodes)

    return [simulation]


def generate_launch_description():


    declared_arguments = [
        DeclareLaunchArgument("simulator", default_value="gazebo"),
        DeclareLaunchArgument("robot_namespace", default_value="robot"),
        DeclareLaunchArgument("meta_description_file_path"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
