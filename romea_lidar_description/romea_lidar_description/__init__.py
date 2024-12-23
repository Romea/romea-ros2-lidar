from ament_index_python.packages import get_package_share_directory

import xacro
import importlib
import yaml


def vendor_module(type):
    try:
        return importlib.import_module("romea_lidar_description."+type)
    except:
        raise RuntimeError(
            f"Lidar of type '{type}'  is unsuported by romea_lidar_description package. "
            'Please check your configuration or contribute to support this kind of sensor.'
        )


def get_lidar_complete_configuration(type, model, rate, resolution):
    return vendor_module(type).get_lidar_complete_configuration(model, rate, resolution)


def get_lidar_specifications_file_path(type, model):
    return vendor_module(type).get_lidar_specifications_file_path(model)


def get_lidar_specifications(type, model):
    return vendor_module(type).get_lidar_specifications(model)


def get_lidar_geometry_file_path(type, model):
    return vendor_module(type).get_lidar_geometry_file_path(model)


def get_lidar_geometry(type, model):
    return vendor_module(type).get_lidar_geometry(model)


def save_lidar_specifications(prefix, lidar_name, configuration):
    configuration_file_path = '/tmp/' + prefix + lidar_name + '_specifications.yaml'

    with open(configuration_file_path, 'w') as f:
        yaml.dump(configuration, f)

    return configuration_file_path


def urdf(prefix, mode, name, type, model, rate, resolution, parent_link, xyz, rpy, ros_namespace):
    specifications = get_lidar_complete_configuration(type, model, rate, resolution)
    specifications_yaml_file = save_lidar_specifications(prefix, name, specifications)
    geometry_yaml_file = get_lidar_geometry_file_path(type, model)

    xacro_file = (
        get_package_share_directory('romea_lidar_description')
        + '/urdf/lidar'
        + specifications['type']
        + '.xacro.urdf'
    )

    if mode == 'simulation':
        mode += '_gazebo_classic'

    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            'prefix': prefix,
            'mode': mode,
            'name': name,
            'sensor_config_yaml_file': specifications_yaml_file,
            'geometry_config_yaml_file': geometry_yaml_file,
            'parent_link': parent_link,
            'xyz': ' '.join(map(str, xyz)),
            'rpy': ' '.join(map(str, rpy)),
            'mesh_visual': str(True),
            'ros_namespace': ros_namespace,
        },
    )

    return urdf_xml.toprettyxml()
