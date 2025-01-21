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
from romea_common_description import DeviceConfiguration as Device
from ament_index_python.packages import get_package_share_directory


def get_sick_model_family(model):
    if 'lms1' in model:
        return 'lms1xx'
    elif 'tim5' in model:
        return 'tim5xx'
    elif 'mrs1' in model:
        return 'mrs1xxx'

    raise RuntimeError(
        f'Sick {model} lidar is not supported by romea_lidar_description package. '
        'Please check your configuration or contribute to support this sensor.'
    )


def get_ouster_model_family(model):
    if 'os1' in model:
        return 'os1'

    raise RuntimeError(
        f'Ouster {model} lidar is not supported by romea_lidar_description package. '
        'Please check your configuration or contribute to support this sensor.'
    )


def get_lidar_family(type, model):
    if type == 'sick':
        return get_sick_model_family(model)
    elif type == 'ouster':
        return get_ouster_model_family(model)
    else:
        raise RuntimeError(
            f"Lidar of type '{type}'  is unsuported by romea_lidar_description package. "
            'Please check your configuration or contribute to support this kind of sensor.'
        )


def get_lidar_specifications_file_path(type, model):
    family = get_lidar_family(type, model)
    pkg_path = get_package_share_directory('romea_lidar_description')
    return f'{pkg_path}/config/{type}_{family}_specifications.yaml'


def get_lidar_specifications(type, model):
    with open(get_lidar_specifications_file_path(type, model)) as f:
        return yaml.safe_load(f)


def get_lidar_geometry_file_path(type, model):
    family = get_lidar_family(type, model)
    pkg_path = get_package_share_directory('romea_lidar_description')
    return f'{pkg_path}/config/{type}_{family}_geometry.yaml'


def get_lidar_geometry(type, model):
    with open(get_lidar_geometry_file_path(type, model)) as f:
        return yaml.safe_load(f)


def get_lidar_specification_units_file_path():
    pkg_path = get_package_share_directory('romea_lidar_description')
    return f'{pkg_path}/config/specifications_units.yaml'


def get_lidar_specification_units():
    with open(get_lidar_specification_units_file_path()) as f:
        return yaml.safe_load(f)


def get_lidar_complete_configuration(lidar_name, lidar_description):

    type = lidar_description["type"]
    model = lidar_description["model"]
    lidar_name = f'{type} {model} lidar called {lidar_name}'
    specifications = get_lidar_specifications(type, model)
    specifications_units = get_lidar_specification_units()

    lidar = Device(lidar_name, specifications, lidar_description, specifications_units)

    configuration = {}
    configuration['model'] = model
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

    return configuration


def urdf(prefix, mode, lidar_name, lidar_description, lidar_location, ros_namespace):

    configuration = get_lidar_complete_configuration(lidar_name, lidar_description)

    configuration_yaml_file = f'/tmp/{prefix}{lidar_name}_urdf_configuration.yaml'

    with open(configuration_yaml_file, 'w') as f:
        yaml.dump({**configuration, **lidar_location}, f)

    geometry_yaml_file = get_lidar_geometry_file_path(
        lidar_description["type"], lidar_description["model"]
    )

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
