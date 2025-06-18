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

import romea_common_description
import romea_lidar_description
from romea_common_meta_bringup import SensorMetaDescription, LaunchFileGenerator


class LIDARMetaDescription(SensorMetaDescription):
    def __init__(self, meta_description_file_path, robot_name=None):
        super().__init__("lidar", meta_description_file_path, robot_name)

    def get_rate(self):
        return self._get_or("rate", "configuration", None)

    def get_azimut_resolution(self):
        return self._get_or("azimut_resolution", "configuration", None)


def load_meta_description(meta_description_file_path, robot_name=None):
    return LIDARMetaDescription(meta_description_file_path, robot_name)


def get_sensor_specifications(meta_description):
    return romea_lidar_description.get_lidar_specifications(meta_description.get_configuration())


def get_sensor_geometry(meta_description):
    return romea_lidar_description.get_lidar_geometry(meta_description.get_configuration())


def get_complete_sensor_configuration(meta_description):
    return romea_lidar_description.get_lidar_complete_configuration(
        meta_description.get_name(), meta_description.get_configuration()
    )


def generate_configuration_file(meta_description, extended):
    configuration = get_complete_sensor_configuration(meta_description)
    units = romea_lidar_description.get_lidar_specification_units()
    return romea_common_description.generate_configuration_file(configuration, units, extended)


def generate_launch_file(meta_description):
    launch_arguments = [{"name": "mode", "default": "live"}]
    namespaces = [meta_description.get_robot_name(), meta_description.get_name()]
    configuration = get_complete_sensor_configuration(meta_description)
    configuration["tf_prefix"] = meta_description.get_urdf_prefix()
    configuration["frame_id"] = meta_description.get_link()

    return LaunchFileGenerator("lidar").generate(
        meta_description.get_launch_file(), launch_arguments, namespaces, configuration
    )


def generate_urdf_description(mode, meta_description):

    return romea_lidar_description.urdf(
        meta_description.get_urdf_prefix(),
        mode,
        meta_description.get_name(),
        meta_description.get_configuration(),
        meta_description.get_location(),
        meta_description.get_full_namespace(),
    )
