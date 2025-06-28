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

import xacro
import yaml
import romea_common_description
# from romea_common_description import get_geometry_file_path
# from romea_common_description import get_specifications_file_path
# from romea_common_description import generate_configuration_file
# from romea_common_description import DeviceConfiguration as Device
from ament_index_python.packages import get_package_share_directory


def get_specifications_file_path(lidar_description):
    return romea_common_description.get_specifications_file_path(
        "romea_lidar_description", lidar_description
    )


def get_specifications(lidar_description):
    with open(get_specifications_file_path(lidar_description)) as f:
        return yaml.safe_load(f)


def get_geometry_file_path(lidar_description):
    return romea_common_description.get_geometry_file_path(
        "romea_lidar_description", lidar_description
    )


def get_geometry(lidar_description):
    with open(get_geometry_file_path(lidar_description)) as f:
        return yaml.safe_load(f)


def get_specification_units_file_path():
    pkg_path = get_package_share_directory('romea_lidar_description')
    return f'{pkg_path}/config/specifications_units.yaml'


def get_specification_units():
    with open(get_specification_units_file_path()) as f:
        return yaml.safe_load(f)


def get_complete_configuration(lidar_name, lidar_description, lidar_location):

    model = lidar_description["model"]
    version = lidar_description["version"]
    manufacturer = lidar_description["manufacturer"]
    lidar_name = f'{manufacturer} {model} {version} lidar called {lidar_name}'
    specifications = get_specifications(lidar_description)
    specifications_units = get_specification_units()

    lidar = romea_common_description.DeviceConfiguration(
        lidar_name, specifications, lidar_description, specifications_units)

    configuration = {}
    configuration["model"] = lidar_description["model"]
    configuration["version"] = lidar_description["version"]
    configuration["manufacturer"] = lidar_description["manufacturer"]
    configuration['rate'] = lidar.get('rate')
    configuration['minimal_azimut_angle'] = lidar.get('minimal_azimut_angle')
    configuration['maximal_azimut_angle'] = lidar.get('maximal_azimut_angle')
    configuration['azimut_resolution'] = lidar.get('azimut_resolution')
    configuration['azimut_angle_std'] = lidar.get('azimut_angle_std')
    configuration['minimal_range'] = lidar.get('minimal_range')
    configuration['maximal_range'] = lidar.get('maximal_range')
    configuration['range_std'] = lidar.get('range_std')
    configuration['samples'] = lidar.get('samples')

    if "lasers" in specifications:
        configuration['lasers'] = lidar.get('lasers')
        configuration['minimal_elevation_angle'] = lidar.get('minimal_elevation_angle')
        configuration['maximal_elevation_angle'] = lidar.get('maximal_elevation_angle')
        configuration['elevation_resolution'] = lidar.get('elevation_resolution')
        configuration['elevation_angle_std'] = lidar.get('elevation_angle_std')

    return {**configuration, **lidar_location}


def generate_configuration_file(configuration, extended):
    units = get_specification_units()
    return romea_common_description.generate_configuration_file(configuration, units, extended)


def generate_urdf_description(
        prefix, mode, lidar_name, lidar_description, lidar_location, ros_namespace
):

    configuration = get_complete_configuration(
        lidar_name, lidar_description, lidar_location
    )

    configuration_yaml_file = f'/tmp/{prefix}{lidar_name}_urdf_configuration.yaml'
    with open(configuration_yaml_file, 'w') as f:
        f.write(generate_configuration_file(configuration, False))

    geometry_yaml_file = get_geometry_file_path(lidar_description)

    package_shared_directory = get_package_share_directory('romea_lidar_description')
    if "lasers" in configuration:
        xacro_file = package_shared_directory + '/urdf/lidar3D.xacro.urdf'
    else:
        xacro_file = package_shared_directory + '/urdf/lidar2D.xacro.urdf'

    if mode == 'simulation':
        mode += '_gazebo_classic'

    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            'prefix': prefix,
            'mode': mode,
            'name': lidar_name,
            'sensor_config_yaml_file': configuration_yaml_file,
            'geometry_config_yaml_file': geometry_yaml_file,
            'mesh_visual': str(True),
            'ros_namespace': ros_namespace,
        },
    )

    return urdf_xml.toprettyxml()
