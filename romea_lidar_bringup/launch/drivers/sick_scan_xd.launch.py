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

import yaml


def launch_setup(context, *args, **kwargs):

    executable = LaunchConfiguration("executable").perform(context)
    executable_namespace = LaunchConfiguration("executable").perform(context)
    configuration_file_path = LaunchConfiguration("configuration_file_path").perform(context)

    driver = LaunchDescription()

    print(f'config_path: {configuration_file_path}')
    with open(configuration_file_path, 'r') as file:
        config_parameters = yaml.safe_load(file)

    parameters = [
        {"nodename": "driver"},
        {"cloud_topic": "cloud"},
        {"laserscan_topic": "scan"},
        {"range_filter_handling": 0},
        {"intensity": False},
        {"intensity_resolution_16bit": False},
        {"use_binary_protocol": True},
        {"timelimit": 5},
        {"sw_pll_only_publish": True},
        {"use_generation_timestamp": True},
        {"start_services": True},
        {"activate_lferec": True},
        {"activate_lidoutputstate": True},
        {"activate_lidinputstate": True},
        {"min_intensity": 0.0},
        {"encoder_mode": -1},
        {"scandatacfg_timingflag": -1},
        {"add_transform_xyz_rpy": "0,0,0,0,0,0"},
        {"add_transform_check_dynamic_updates": False},
        {"message_monitoring_enabled": True},
        {"read_timeout_millisec_default": 5000},
        {"read_timeout_millisec_startup": 120000},
        {"read_timeout_millisec_kill_node": 150000},
        {"client_authorization_pw": "F4724744"},
        {"ros_qos": -1},
        {"tick_to_timestamp_mode": 0},
        config_parameters,
    ]

    driver_node = Node(
        package="sick_scan_xd",
        executable=executable,
        name="driver",
        namespace=executable_namespace,
        output="screen",
        parameters=parameters,
    )

    driver.add_action(driver_node)

    return [driver]


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument("executable"),
        DeclareLaunchArgument("executable_namespace", default_value=""),
        DeclareLaunchArgument("component_container", default_value=""),
        DeclareLaunchArgument("configuration_file_path"),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
