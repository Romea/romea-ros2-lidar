# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch_ros.actions import PushRosNamespace
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from romea_common_bringup import device_link_name
from romea_lidar_bringup import LIDARMetaDescription


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_meta_description(context):

    meta_description_file_path = LaunchConfiguration(
        "meta_description_file_path"
    ).perform(context)

    return LIDARMetaDescription(meta_description_file_path)


def launch_setup(context, *args, **kwargs):

    robot_namespace = get_robot_namespace(context)
    meta_description = get_meta_description(context)

    if not meta_description.has_driver_configuration():
        return []

    lidar_name = meta_description.get_name()
    lidar_namespace = str(meta_description.get_namespace() or "")

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("romea_lidar_bringup"),
                        "launch",
                        "drivers/" + meta_description.get_driver_pkg() + ".launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "ip": meta_description.get_ip(),
            "port": str(meta_description.get_port(meta_description)),
            "lidar_model": meta_description.get_model(meta_description),
            "rate": str(meta_description.get_rate(meta_description) or ""),
            "resolution": str(meta_description.get_resolution_deg(meta_description) or ""),
            "frame_id": device_link_name(robot_namespace, lidar_name),
        }.items(),
    )

    return [
        GroupAction(
            actions=[
                PushRosNamespace(robot_namespace),
                PushRosNamespace(lidar_namespace),
                PushRosNamespace(lidar_name),
                driver,
            ]
        )
    ]


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("meta_description_file_path"))

    declared_arguments.append(
        DeclareLaunchArgument("robot_namespace", default_value="")
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
