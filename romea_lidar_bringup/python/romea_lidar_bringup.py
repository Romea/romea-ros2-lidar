#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
import romea_lidar_description
import yaml

def get_lidar_name(meta_description):
    return meta_description["name"]

def has_lidar_driver_configuration(meta_description):
    return "driver" in meta_description

def get_lidar_driver_pkg(meta_description):
    return meta_description["driver"]["pkg"]

def get_lidar_ip(meta_description):
    return meta_description["driver"]["ip"]

def get_lidar_port(meta_description):
    return meta_description["driver"]["port"]

def get_lidar_type(meta_description):
    return meta_description["configuration"]["type"]

def get_lidar_model(meta_description):
    return meta_description["configuration"]["model"]

def get_lidar_rate(meta_description):
    return meta_description["configuration"].get("rate", None)

def get_lidar_resolution(meta_description):
    return meta_description["configuration"].get("resolution", None)

def get_lidar_parent_link(meta_description):
    return meta_description["geometry"]["parent_link"]

def get_lidar_xyz(meta_description):
    return meta_description["geometry"]["xyz"]

def get_lidar_rpy(meta_description):
    return meta_description["geometry"]["rpy"]

def urdf_description(prefix, meta_description_filename):

    with open(meta_description_filename) as f:
        meta_description = yaml.safe_load(f)

    return romea_lidar_description.urdf(
        prefix,
        get_lidar_name(meta_description),
        get_lidar_type(meta_description),
        get_lidar_model(meta_description),
        get_lidar_rate(meta_description),
        get_lidar_resolution(meta_description),
        get_lidar_parent_link(meta_description),
        get_lidar_xyz(meta_description),
        get_lidar_rpy(meta_description),
    )
