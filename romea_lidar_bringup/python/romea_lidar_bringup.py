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


from romea_common_bringup import MetaDescription, robot_urdf_prefix, device_namespace
import romea_lidar_description
from numpy import radians, deg2rad


class LIDARMetaDescription:
    def __init__(self, meta_description_file_path):
        self.meta_description = MetaDescription(
            "lidar", meta_description_file_path)

    def get_name(self):
        return self.meta_description.get("name")

    def get_namespace(self):
        return self.meta_description.get_or("namespace", None)

    def has_driver_configuration(self):
        return self.meta_description.exists("driver")

    def get_driver_pkg(self):
        return self.meta_description.get("pkg", "driver")

    def get_driver_ip(self):
        return self.meta_description.get("ip", "driver")

    def get_driver_port(self):
        return self.meta_description.get("port", "driver")

    def get_type(self):
        return self.meta_description.get("type", "configuration")

    def get_model(self):
        return self.meta_description.get("model", "configuration")

    def get_rate(self):
        return self.meta_description.get_or("rate", "configuration", None)

    def get_resolution_deg(self):
        return self.meta_description.get_or("resolution", "configuration", None)

    def get_resolution_rad(self):
        return deg2rad(self.get_resolution_deg())

    def get_parent_link(self):
        return self.meta_description.get("parent_link", "geometry")

    def get_xyz(self):
        return self.meta_description.get("xyz", "geometry")

    def get_rpy_deg(self):
        return self.meta_description.get("rpy", "geometry")

    def get_rpy_rad(self):
        return radians(self.get_rpy_deg()).tolist()


def load_meta_description(meta_description_file_path):
    return LIDARMetaDescription(meta_description_file_path)


def get_sensor_specifications(meta_description):
    return romea_lidar_description.get_lidar_specifications(
        meta_description.get_type(), meta_description.get_model()
    )


def get_sensor_geometry(meta_description):
    return romea_lidar_description.get_lidar_family_geometry(
        meta_description.get_type(), meta_description.get_model()
    )


def urdf_description(robot_namespace, mode, meta_description_file_path):

    meta_description = LIDARMetaDescription(meta_description_file_path)

    ros_namespace = device_namespace(
        robot_namespace,
        meta_description.get_namespace(),
        meta_description.get_name()
    )
    return romea_lidar_description.urdf(
        robot_urdf_prefix(robot_namespace),
        mode,
        meta_description.get_name(),
        meta_description.get_type(),
        meta_description.get_model(),
        meta_description.get_rate(),
        meta_description.get_resolution_deg(),
        meta_description.get_parent_link(),
        meta_description.get_xyz(),
        meta_description.get_rpy_rad(),
        ros_namespace
    )
