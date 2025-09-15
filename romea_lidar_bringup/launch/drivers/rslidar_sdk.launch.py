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
from launch_ros.actions import Node

import yaml


def launch_setup(context, *args, **kwargs):
    package = LaunchConfiguration("package").perform(context)
    executable = LaunchConfiguration("executable").perform(context)
    config_path = LaunchConfiguration("config_path").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)
    lidar_model = LaunchConfiguration("lidar_model").perform(context)
    lidar_name = LaunchConfiguration("lidar_name").perform(context)
    # resolution = LaunchConfiguration("resolution").perform(context)
    # rate = LaunchConfiguration("rate").perform(context)

    driver = LaunchDescription()

    print(f"config_path: {config_path}")
    with open(config_path, "r") as file:
        config_parameters = yaml.safe_load(file)

    # fmt: off
    params = {
        "common": {
            "msg_source": 1,               # 0: not use Lidar
                                           # 1: packet message comes from online Lidar
                                           # 2: packet message comes from ROS or ROS2
                                           # 3: packet message comes from Pcap file
            "send_packet_ros": False,      # true: Send packets through ROS or ROS2 (for recording)
            "send_point_cloud_ros": True,  # true: Send point cloud through ROS or ROS2
        },
        "lidar": [
            {
                "driver": {
                    #  LiDAR type - RS16, RS32, RSBP, RSAIRY, RSHELIOS, RSHELIOS_16P, RS128, RS80,
                    #  RS48, RSP128, RSP80, RSP48, RSM1, RSM1_JUMBO, RSM2, RSM3, RSE1, RSMX.
                    "lidar_type": "",       
                                                       
                    "msop_port": 6699,        # Msop port of lidar
                    "difop_port": 7788,       # Difop port of lidar
                    "imu_port": 0,            # IMU port of lidar(only for RSAIRY, RSE1), 0 means
                                              # no imu. If you want to use IMU, please first set
                                              # ENABLE_IMU_DATA_PARSE to ON in CMakeLists.txt 

                    "user_layer_bytes": 0,    # Bytes of user layer. If 0, disabled
                    "tail_layer_bytes": 0,    # Bytes of tail layer. If 0, disabled


                    "min_distance": 0.2,      # Minimum distance of point cloud
                    "max_distance": 200,      # Maximum distance of point cloud
                    "use_lidar_clock": True,  # true: Use the lidar clock as the message timestamp
                                              # false: Use the system clock as the timestamp
                    "dense_points": False,    # true: discard NAN points; false: reserve NAN points
                          
                    "ts_first_point": False,  # true: time-stamp point cloud with the first point; 
                                              # false: with the last point;   
                                              # these parameters are used from mechanical lidar

                    "start_angle": 0,         # Start angle of point cloud
                    "end_angle": 360,         # End angle of point cloud

                    #  When msg_source is 3, the following parameters will be used
                    "pcap_repeat": True,             # true: The pcap bag will repeat play   
                    "pcap_rate": 1.0,                # Rate to read the pcap file
                    "pcap_path": "/tmp/lidar.pcap",  # The path of pcap file
                },
                "ros": {
                    "ros_frame_id": frame_id,
                    "ros_recv_packet_topic": "packets",
                    "ros_send_packet_topic": "packets",
                    "ros_send_imu_data_topic": "imu",
                    "ros_send_point_cloud_topic": "points",
                    "ros_queue_length": 20,
                }
            },
        ]
    }
    # fmt: on

    apply_specific_config(lidar_model, params)

    param_file = f"/tmp/{lidar_name}_config.yaml"
    with open(param_file, "w") as f:
        yaml.dump(params, f)

    driver_node = Node(
        package=package,
        executable=executable,
        exec_name=lidar_name,
        name=lidar_name,
        output="screen",
        parameters=[{"config_path": param_file}],
    )

    driver.add_action(driver_node)

    return [driver]


def apply_specific_config(model: str, params: dict):
    driver = params["lidar"][0]["driver"]

    if model == "airy":
        driver["lidar_type"] = "RSAIRY"
        driver["imu_port"] = 6688
        driver["min_distance"] = 0.1


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("executable"),
        DeclareLaunchArgument("config_path"),
        DeclareLaunchArgument("frame_id"),
        DeclareLaunchArgument("lidar_model"),
        DeclareLaunchArgument("lidar_name"),
        DeclareLaunchArgument("rate"),
        DeclareLaunchArgument("resolution"),
        DeclareLaunchArgument("package", default_value="rslidar_sdk"),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
