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

from romea_lidar_bringup import (
    get_lidar_name,
    has_lidar_driver_configuration,
    get_lidar_driver_pkg,
    get_lidar_ip,
    get_lidar_port,
    get_lidar_model,
    get_lidar_rate,
    get_lidar_resolution,
)

import yaml


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_meta_description(context):

    meta_description_filename = LaunchConfiguration(
        "meta_description_filename"
    ).perform(context)

    with open(meta_description_filename) as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):

    robot_namespace = get_robot_namespace(context)
    meta_description = get_meta_description(context)

    if not has_lidar_driver_configuration(meta_description):
        return []

    lidar_name = get_lidar_name(meta_description)
    if robot_namespace != "":
        frame_id = robot_namespace + "_" + lidar_name + "_link"
    else:
        frame_id = lidar_name + "_link"

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("romea_lidar_bringup"),
                        "launch",
                        "drivers/"
                        + get_lidar_driver_pkg(meta_description)
                        + ".launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "ip": get_lidar_ip(meta_description),
            "port": str(get_lidar_port(meta_description)),
            "lidar_model": get_lidar_model(meta_description),
            "rate": str(get_lidar_rate(meta_description) or ""),
            "resolution": str(get_lidar_resolution(meta_description) or ""),
            "frame_id": frame_id,
        }.items(),
    )

    return [
        GroupAction(
            actions=[
                PushRosNamespace(robot_namespace),
                PushRosNamespace(lidar_name),
                driver,
            ]
        )
    ]


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("meta_description_filename"))

    declared_arguments.append(
        DeclareLaunchArgument("robot_namespace", default_value="")
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
