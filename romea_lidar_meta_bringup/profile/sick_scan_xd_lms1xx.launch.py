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

import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    ip = LaunchConfiguration("ip").perform(context)
    port = LaunchConfiguration("port").perform(context)
    auto_configuration = LaunchConfiguration("auto_configuration").perform(context)

    mode = LaunchConfiguration("mode").perform(context)
    rate = LaunchConfiguration("rate").perform(context)
    minimal_azimut_angle = LaunchConfiguration("minimal_azimut_angle").perform(context)
    maximal_azimut_angle = LaunchConfiguration("maximal_azimut_angle").perform(context)
    azimut_resolution = LaunchConfiguration("azimut_resolution").perform(context)
    minimal_range = LaunchConfiguration("minimal_range").perform(context)
    maximal_range = LaunchConfiguration("maximal_range").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)

    launch = LaunchDescription()

    if mode == "live":

        parameters = {
            "scanner_type": "sick_lms_1xx",
            "frame_id": frame_id,
            "hostname": ip,
            "port": int(port),
            "min_ang": float(minimal_azimut_angle)*math.pi/180,
            "max_ang": float(maximal_azimut_angle)*math.pi/180,
            "range_min":  float(minimal_range),
            "range_max": float(maximal_range),
            # according sick_lms_1xx.launch from sick_scan_xd we need to specify these parameters
            "encoder_mode": -1,
            "intensity": False,
            "intensity_resolution_16bit": False,
            "sw_pll_only_publish": True,
            # common to all launch
            "range_filter_handling": 0,
            "cloud_topic": "cloud",
            "laserscan_topic": "scan",
            "use_generation_timestamp": True,
            "ros_qos": -1,
            "tf_base_frame_id": "foo",
            "tf_base_lidar_xyz_rpy": "0,0,0,0,0,0",
            "tf_publish_rate": 0.0,
            # equal to default value
            # use_binary_protocol: True
            # timelimit: 5
            # min_intensity: 0.0
            # tick_to_timestamp_mode: 0
            # start_services: True
            # activate_lferec: True
            # activate_lidoutputstate: True
            # activate_lidinputstate: True
            # scandatacfg_timingflag: -1
            # add_transform_xyz_rpy: "0,0,0,0,0,0"
            # add_transform_check_dynamic_updates: False
            # message_monitoring_enabled: True
            # read_timeout_millisec_default: 5000
            # read_timeout_millisec_startup: 120000
            # read_timeout_millisec_kill_node: 150000
            # client_authorization_pw: "F4724744"
        }

        if auto_configuration:
            parameters["ang_res"] = float(azimut_resolution)*math.pi/180
            parameters["scan_freq"] = float(rate)

        launch.add_action(
            Node(
                package="sick_scan_xd",
                executable="sick_generic_caller",
                name="driver",
                parameters=[parameters]
            )
        )

    return [launch]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("ip"),
            DeclareLaunchArgument("port"),
            DeclareLaunchArgument("auto_configuration", default_value="false"),
            OpaqueFunction(function=launch_setup)
        ]
    )
