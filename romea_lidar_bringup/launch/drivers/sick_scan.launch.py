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


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("executable"),
        DeclareLaunchArgument("config_path"),
        DeclareLaunchArgument("frame_id"),
        DeclareLaunchArgument("lidar_model"),
        DeclareLaunchArgument("lidar_name"),
        DeclareLaunchArgument("rate"),
        DeclareLaunchArgument("resolution"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("romea_lidar_bringup"),
                    "launch",
                    "drivers/sick_scan_xd.launch.py",
                ])
            ]),
            launch_arguments={
                "executable": LaunchConfiguration("executable"),
                "config_path": LaunchConfiguration("config_path"),
                "frame_id": LaunchConfiguration("frame_id"),
                "lidar_model": LaunchConfiguration("lidar_model"),
                "lidar_name": LaunchConfiguration("lidar_name"),
                "rate": LaunchConfiguration("rate"),
                "resolution": LaunchConfiguration("resolution"),
                "package": LaunchConfiguration("sick_scan"),  # difference with sick_scan_xd
            }.items(),
        )
    ])
