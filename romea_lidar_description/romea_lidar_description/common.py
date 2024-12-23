from ament_index_python.packages import get_package_share_directory

import yaml


def get_lidar_specifications_file_path(type, model_or_family):
    return (
        get_package_share_directory('romea_lidar_description')
        + '/config/'
        + type
        + '_'
        + model_or_family
        + '_specifications.yaml'
    )


def get_lidar_specifications(type, model_or_family):
    with open(get_lidar_specifications_file_path(type, model_or_family)) as f:
        return yaml.safe_load(f)


def get_lidar_geometry_file_path(type, model_or_family):
    pkg_path = get_package_share_directory('romea_lidar_description')
    return f'{pkg_path}/config/{type}_{model_or_family}_geometry.yaml'


def get_lidar_geometry(type, model_or_family):
    with open(get_lidar_geometry_file_path(type, model_or_family)) as f:
        return yaml.safe_load(f)
