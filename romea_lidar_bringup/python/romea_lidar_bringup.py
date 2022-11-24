#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
import romea_lidar_description
import yaml


def urdf_description(prefix, description_yaml_file):

    with open(description_yaml_file) as f:
        device = yaml.safe_load(f)

    return romea_lidar_description.urdf(
        prefix,
        device["name"],
        device["configuration"]["type"],
        device["configuration"]["model"],
        device["configuration"].get("rate", None),
        device["configuration"].get("resolution", None),
        device["geometry"]["parent_link"],
        device["geometry"]["xyz"],
        device["geometry"]["rpy"],
    )
