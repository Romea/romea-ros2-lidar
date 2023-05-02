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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import romea_lidar_description
import math


def launch_setup(context, *args, **kwargs):

    ip = LaunchConfiguration("ip").perform(context)
    port = LaunchConfiguration("port").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)
    lidar_model = LaunchConfiguration("lidar_model").perform(context)
    resolution = LaunchConfiguration("resolution").perform(context)
    rate = LaunchConfiguration("rate").perform(context)

    if rate == "":
        rate = None
    else:
        rate = int(rate)

    if resolution == "":
        resolution = None
    else:
        resolution = float(resolution)

    driver = LaunchDescription()

    parameters = [
        {"nodename": "driver"},
        {"hostname": ip},
        {"port": port},
        {"cloud_topic": "cloud"},
        {"frame_id": frame_id},
        {"range_filter_handling": 0},
        {"intensity": False},
        {"intensity_resolution_16bit": False},
        {"use_binary_protocol": True},
        {"timelimit": 5},
        {"sw_pll_only_publish": True},
        {"use_generation_timestamp": True},
        # Use the lidar generation timestamp (true, default) or send timestamp (false)
        # for the software pll converted message timestamp
        {"start_services": True},
        # start ros service for cola commands
        {"activate_lferec": True},
        # activate field monitoring by lferec message
        {"activate_lidoutputstate": True},
        # activate field monitoring by lidoutputstate messages
        {"activate_lidinputstate": True},
        # activate field monitoring by lidinputstate messages
        {"min_intensity": 0.0},
        # Set range of LaserScan messages to infinity, if intensity < min_intensity (default: 0)
        {"add_transform_xyz_rpy": "0,0,0,0,0,0"},
        {"message_monitoring_enabled": True},
        # Enable message monitoring with reconnect+reinit in case of timeouts
        {"read_timeout_millisec_default": 5000},
        # 5 sec read timeout in operational mode (measurement mode), default: 5000 milliseconds
        {"read_timeout_millisec_startup": 120000},
        # 120 sec read timeout during startup (sensor may be starting up,
        #  which can take up to 120 sec.), default: 120000 milliseconds
        {"read_timeout_millisec_kill_node": 150000},
        # 150 sec pointcloud timeout, ros node will be killed if no point cloud published
        # within the last 150 sec., default: 150000 milliseconds
        {"client_authorization_pw": "F4724744"},
        # Default password for client authorization
    ]

    if "lms1" in lidar_model:
        configuration = romea_lidar_description.sick_lms1xx_configuration(
            lidar_model, rate, resolution
        )
        parameters.append(
            {"min_ang": configuration["minimal_azimut_angle"] / 180.0 * math.pi}
        )
        parameters.append(
            {"max_ang": configuration["maximal_azimut_angle"] / 180.0 * math.pi}
        )
        parameters.append({"range_min": configuration["minimal_range"]})
        parameters.append({"range_max": configuration["maximal_range"]})
        parameters.append({"scanner_type": "sick_lms_1xx"})

    if "tim5" in lidar_model:
        configuration = romea_lidar_description.sick_tim5xx_configuration(
            lidar_model, rate, resolution
        )
        parameters.append(
            {"min_ang": configuration["minimal_azimut_angle"] / 180.0 * math.pi}
        )
        parameters.append(
            {"max_ang": configuration["maximal_azimut_angle"] / 180.0 * math.pi}
        )
        parameters.append({"range_min": configuration["minimal_range"]})
        parameters.append({"range_max": configuration["maximal_range"]})
        parameters.append({"scanner_type": "sick_tim_5xx"})

    driver_node = Node(
        package="sick_scan",
        executable="sick_generic_caller",
        name="driver",
        output="screen",
        parameters=parameters,
    )

    driver.add_action(driver_node)

    return [driver]


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("ip"))
    declared_arguments.append(DeclareLaunchArgument("port"))
    declared_arguments.append(DeclareLaunchArgument("frame_id"))
    declared_arguments.append(DeclareLaunchArgument("lidar_model"))
    declared_arguments.append(DeclareLaunchArgument("rate"))
    declared_arguments.append(DeclareLaunchArgument("resolution"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
