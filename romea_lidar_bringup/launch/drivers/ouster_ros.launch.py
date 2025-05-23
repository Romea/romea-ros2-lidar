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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

import yaml


def launch_setup(context, *args, **kwargs):
    # package = LaunchConfiguration("package").perform(context)
    # executable = LaunchConfiguration("executable").perform(context)
    config_path = LaunchConfiguration("config_path").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)
    # lidar_model = LaunchConfiguration("lidar_model").perform(context)
    # lidar_name = LaunchConfiguration("lidar_name").perform(context)
    resolution = LaunchConfiguration("resolution").perform(context)
    rate = LaunchConfiguration("rate").perform(context)

    if rate == "":
        rate = "10"

    if resolution == "":
        nb_samples = 512
    else:
        nb_samples = round(360. / float(resolution))

    driver = LaunchDescription()

    print(f"config_path: {config_path}")
    with open(config_path, "r") as file:
        config_parameters = yaml.safe_load(file)

    ouster_params = {
        # sensor_hostname[required]: hostname or IP address of the sensor (IP4 or IP6).
        "sensor_hostname": "",
        # udp_dest[optional]: hostname or multicast group IP where the sensor will
        # send UDP data packets.
        "udp_dest": "",
        # mtp_dest[optional]: hostname IP address for receiving data packets via
        # multicast, by default it is INADDR_ANY, so packets will be received on
        # first available network interface.
        "mtp_dest": "",
        # mtp_main[optional]: if true, then configure and reinit the sensor,
        # otherwise start client with active configuration of sensor
        "mtp_main": False,
        # lidar_mode[optional]: resolution and rate; possible values: { 512x10,
        # 512x20, 1024x10, 1024x20, 2048x10, 4096x5 }. Leave empty to remain on
        # current the lidar mode.
        "lidar_mode": f"{nb_samples}x{rate}",
        # timestamp_mode[optional]: method used to timestamp measurements; possible
        # "values":
        # - TIME_FROM_INTERNAL_OSC
        # - TIME_FROM_SYNC_PULSE_IN
        # - TIME_FROM_PTP_1588
        # - "TIME_FROM_ROS_TIME": This option uses the time of reception of first
        #                       packet of a LidarScan as the timestamp of the IMU,
        #                       PointCloud2 and LaserScan messages.
        "timestamp_mode": "TIME_FROM_PTP_1588",
        # ptp_utc_tai_offset[optional]: UTC/TAI offset in seconds to apply when
        # TIME_FROM_PTP_1588 timestamp mode is used.
        "ptp_utc_tai_offset": -37.0,
        # udp_profile_lidar[optional]: lidar packet profile; possible values:
        # - "LEGACY": not recommended
        # - RNG19_RFL8_SIG16_NIR16
        # - RNG15_RFL8_NIR8
        # - RNG19_RFL8_SIG16_NIR16_DUAL
        # - FUSA_RNG15_RFL8_NIR8_DUAL
        "udp_profile_lidar": "",
        # metadata[optional]: path to save metadata file to, if left empty a file
        # with the sensor hostname or ip will be crearted in ~/.ros folder.
        "metadata": "",
        # lidar_port[optional]: port value should be in the range [0, 65535]. If you
        # use 0 as the port value then the first avaliable port number will be
        # assigned.
        "lidar_port": 0,
        # imu_port[optional]: port value should be in the range [0, 65535]. If you
        # use 0 as the port value then the first avaliable port number will be
        # assigned.
        "imu_port": 0,
        # sensor_frame[optional]: name to use when referring to the sensor frame.
        "sensor_frame": frame_id,
        # lidar_frame[optional]: name to use when referring to the lidar frame.
        "lidar_frame": frame_id + "_lidar",
        # imu_frame[optional]: name to use when referring to the imu frame.
        "imu_frame": frame_id + "_imu",
        # point_cloud_frame[optional]: which frame of reference to use when
        # generating PointCloud2 or LaserScan messages, select between the values of
        # lidar_frame and sensor_frame.
        "point_cloud_frame": frame_id,
        # pub_static_tf[optional]: when this flag is set to True, the driver will
        # broadcast the TF transforms for the imu/sensor/lidar frames. Prevent the
        # driver from broadcasting TF transforms by setting this parameter to False.
        "pub_static_tf": True,
        # proc_mask[optional]: use any combination of the 6 flags IMG, PCL, IMU, SCAN
        # RAW and TLM to enable or disable their respective messages.
        # "proc_mask": "IMU|PCL|SCAN|IMG|RAW|TLM",
        "proc_mask": "IMU|PCL",
        # scan_ring[optional]: use this parameter in conjunction with the SCAN flag
        # to select which beam of the LidarScan to use when producing the LaserScan
        # message. Choose a value the range [0, sensor_beams_count).
        "scan_ring": 0,
        # use_system_default_qos[optional]: if false, data are published with sensor
        # data QoS. This is preferrable for production but default QoS is needed for
        # rosbag. "See": https://github.com/ros2/rosbag2/issues/125
        "use_system_default_qos": False,
        # point_type[optional]: choose from: {original, native, xyz, xyzi, o_xyzi,
        #                                     yzir}
        # Here is a breif description of each "option":
        #  - "original": This uses the original point representation ouster_ros::Point
        #          of the ouster-ros driver.
        #  - "native": directly maps all fields as published by the sensor to an
        #          equivalent point cloud representation with the additon of ring
        #          and timestamp fields.
        #  - "xyz": "the simplest point type, only has {x, y", z}
        #  - "xyzi": same as xyz point type but adds intensity (signal) field. this
        #          type is not compatible with the low data profile.
        #  - "o_xyzi": same as xyzi point type but doesn't add the extra 4 padding bytes.
        #  - "xyzir": same as xyzi type but adds ring (channel) field.
        #          this type is same as Velodyne point cloud type
        #          this type is not compatible with the low data profile.
        # for more details about the fields of each point type and their data refer
        # to the following header "files":
        # - ouster_ros/os_point.h
        # - ouster_ros/sensor_point_types.h
        # - ouster_ros/common_point_types.h.
        # "point_type": "original",
        "point_type": "xyz",
        # azimuth window start[optional]: values range [0, 360000] millidegrees
        "azimuth_window_start": 0,
        # azimuth_window_end[optional]: values range [0, 360000] millidegrees
        "azimuth_window_end": 360000,
        # persist_config[optional]: request the sensor to persist settings
        "persist_config": False,
        # attempt_config[optional]: attempting to reconnect to the sensor after
        # connection loss or sensor powered down
        "attempt_reconnect": False,
        # dormant_period_between_reconnects[optional]: wait time in seconds between
        # reconnection attempts
        "dormant_period_between_reconnects": 1.0,
        # max_failed_reconnect_attempts[optional]: maximum number of attempts trying
        # to communicate with the sensor. Counter resets upon successful connection.
        "max_failed_reconnect_attempts": 2147483647,
        # organized[optional]: whether to generate an organized point cloud. default
        # is organized.
        "organized": True,
        # destagger[optional]: enable or disable point cloud destaggering, default enabled.
        "destagger": True,
        # min_range[optional]: minimum lidar range to consider (meters).
        "min_range": 0.0,
        # max_range[optional]: maximum lidar range to consider (meters).
        "max_range": 1000.0,
        # v_reduction[optional]: vertical beam reduction; available options: {1, 2, 4, 8, 16}.
        "v_reduction": 1,
        # min_scan_valid_columns_ratio[optional]: The minimum ratio of valid columns for
        # processing the LidarScan [0, 1]. default is 0%
        "min_scan_valid_columns_ratio": 0.0,
    }

    # config_parameters will override default values of ouster_params
    ouster_params |= config_parameters

    param_file = "/tmp/ouster_config.yaml"

    with open(param_file, "w") as f:
        full_dict = {"/**": {"ros__parameters": ouster_params}}
        yaml.dump(full_dict, f)

    launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ouster_ros"),
                        "launch",
                        "driver.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "params_file": param_file,
            "viz": "false",
            "ouster_ns": "",
        }.items(),
    )

    driver.add_action(launch)

    return [driver]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("executable"),
        DeclareLaunchArgument("config_path"),
        DeclareLaunchArgument("frame_id"),
        DeclareLaunchArgument("lidar_model"),
        DeclareLaunchArgument("lidar_name"),
        DeclareLaunchArgument("rate"),
        DeclareLaunchArgument("resolution"),
        DeclareLaunchArgument("package", default_value="ouster_ros"),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
