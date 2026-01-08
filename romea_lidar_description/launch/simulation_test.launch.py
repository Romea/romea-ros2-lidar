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


from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from romea_lidar_description import generate_urdf_description


def check_pkg_exists(pkg_name):
    return pkg_name in get_packages_with_prefixes()


def launch_setup(context, *args, **kwargs):

    mode = f'simulation_{LaunchConfiguration("simulator").perform(context)}'
    prefix = "robot_"
    name = "lidar"

    description = {
        "manufacturer": LaunchConfiguration("manufacturer").perform(context),
        "model": LaunchConfiguration("model").perform(context),
        "version": LaunchConfiguration("version").perform(context),
        "rate": int(LaunchConfiguration("rate").perform(context)),
    }

    print(description)

    location = {
        "parent_link": "base_link",
        "xyz": [0.0, 0.0, 0.0],
        "rpy": [0.0, 0.0, 1.0]
    }

    ros_namespace = "robot/lidar"
    standalone = True

    with open("/tmp/urdf", "w") as file:
        file.write(
            generate_urdf_description(
                prefix, mode, name, description, location, ros_namespace, standalone
            )
        )

    simulation = LaunchDescription()

    if check_pkg_exists("gazebo_ros"):

        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("gazebo_ros")
                + "/launch/gazebo.launch.py"
            ),
        )

        simulation.add_action(gazebo)

        spawn_entity = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_gps",
            output="screen",
            arguments=["-file", "/tmp/urdf", "-entity", "gps"],
        )

        simulation.add_action(spawn_entity)

    if check_pkg_exists("ros_gz"):

        gazebo_gui = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("ros_gz_sim")
                + "/launch/gz_sim.launch.py"
            ),
            launch_arguments={
                'gz_args': '-g',
                'on_exit_shutdown': 'True'
            }.items()
        )

        simulation.add_action(gazebo_gui)

        world_sdf_file = (
            get_package_share_directory("romea_simulation_gazebo_worlds") + "worlds/gz_empty.sdf"
        )

        gazebo_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("ros_gz_sim")
                + "/launch/gz_server.launch.py"
            ),
            launch_arguments={
                'world_sdf_file': world_sdf_file,
                'world_sdf_string': 'world',
            }.items()
        )

        simulation.add_action(gazebo_server)

        spawn_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory('ros_gz_sim')
                + "/launch/gz_spawn_model.launch.py"
            ),
            launch_arguments=[
                ('file', '/tmp/urdf'),
                ('entity_name', 'lidar'),
            ],
        )

        simulation.add_action(spawn_lidar)

        ros_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='lidar_bridge',
            output='screen',
            arguments=[
                '/robot/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/robot/lidar/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                '/robot/lidar/points@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/robot/lidar/points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            ]
        )

        simulation.add_action(ros_bridge)

    return [simulation]


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument("simulator", default_value="gazebo"),
        DeclareLaunchArgument("manufacturer", default_value=""),
        DeclareLaunchArgument("model", default_value=""),
        DeclareLaunchArgument("version", default_value=""),
        DeclareLaunchArgument("rate", default_value="25"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
