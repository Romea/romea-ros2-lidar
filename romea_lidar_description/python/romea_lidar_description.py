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


def get_lidar_complete_configuration(type, model, rate, resolution):

    lidar_name = f'{type} {model} lidar'
    specifications = get_lidar_specifications(type, model)
    specifications_units = get_lidar_specification_units()

    user_configuration = {}
    user_configuration["model"] = model
    if rate is not None:
        user_configuration["rate"] = rate
    if resolution is not None:
        user_configuration["azimut_angle_increment"] = resolution

    lidar = Device(lidar_name, specifications, user_configuration, specifications_units)

    configuration = {}
    configuration['type'] = lidar.get('type')
    configuration['rate'] = lidar.get('rate')
    configuration['minimal_azimut_angle'] = lidar.get('minimal_azimut_angle')
    configuration['maximal_azimut_angle'] = lidar.get('maximal_azimut_angle')
    configuration['azimut_angle_increment'] = lidar.get('azimut_angle_increment')
    configuration['azimut_angle_std'] = lidar.get('azimut_angle_std')
    configuration['minimal_range'] = lidar.get('minimal_range')
    configuration['maximal_range'] = lidar.get('maximal_range')
    configuration['range_std'] = lidar.get('range_std')
    configuration['samples'] = lidar.get('samples')

    if lidar.get('type') == "3D":
        configuration['minimal_elevation_angle'] = lidar.get('minimal_elevation_angle')
        configuration['maximal_elevation_angle'] = lidar.get('maximal_elevation_angle')
        configuration['elevation_angle_increment'] = lidar.get('elevation_angle_increment')
        configuration['elevation_angle_std'] = lidar.get('elevation_angle_std')
        configuration['lasers'] = lidar.get('lasers')

    return configuration


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
