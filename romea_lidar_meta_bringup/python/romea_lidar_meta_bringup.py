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


from romea_common_meta_bringup import (
    MetaDescription,
    robot_urdf_prefix,
    device_namespace,
    device_link_name,
)

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

    def get_driver_package(self):
        return self.meta_description.get("package", "driver")

    def get_driver_executable(self):
        return self.meta_description.get("executable", "driver")

    def get_driver_parameters(self):
        return self.meta_description.get("parameters", "driver")

    def get_configuration(self):
        return self.meta_description.get("configuration")

    def get_type(self):
        return self.meta_description.get("type", "configuration")

    def get_model(self):
        return self.meta_description.get("model", "configuration")

    def get_rate(self):
        return self.meta_description.get_or("rate", "configuration", None)

    def get_azimut_resolution_deg(self):
        return self.meta_description.get_or("azimut_resolution", "configuration", None)

    def get_azimut_resolution_rad(self):
        return deg2rad(self.get_azimut_resolution_deg())

    def get_geometry(self):
        return self.meta_description.get("geometry")

    def get_parent_link(self):
        return self.meta_description.get("parent_link", "geometry")

    def get_xyz(self):
        return self.meta_description.get("xyz", "geometry")

    def get_rpy_deg(self):
        return self.meta_description.get("rpy", "geometry")

    def get_rpy_rad(self):
        return radians(self.get_rpy_deg()).tolist()

    def get_records(self):
        return self.meta_description.get_or("records", None,  {})

    def get_bridge(self):
        return self.meta_description.get_or("bridge", None,  {})


def load_meta_description(meta_description_file_path):
    return LIDARMetaDescription(meta_description_file_path)


def get_sensor_specifications(meta_description):
    return romea_lidar_description.get_lidar_specifications(
        meta_description.get_type(), meta_description.get_model()
    )


def get_sensor_geometry(meta_description):
    return romea_lidar_description.get_lidar_geometry(
        meta_description.get_type(), meta_description.get_model()
    )


def get_complete_sensor_configuration(meta_description):
    return romea_lidar_description.get_lidar_complete_configuration(
        meta_description.get_name(), meta_description.get_configuration()
    )


def get_complete_driver_parameters(meta_description, robot_namespace):

    lidar_configuration = get_complete_sensor_configuration(meta_description)
    frame_id = device_link_name(robot_namespace, meta_description.get_name())

    executable = meta_description.get_driver_executable()
    parameters = meta_description.get_driver_parameters()
    parameters["frame_id"] = frame_id

    if executable == "sick_generic_caller":
        parameters["framerate"] = lidar_configuration["rate"]
        parameters["min_ang"] = deg2rad(lidar_configuration["minimal_azimut_angle"])
        parameters["max_ang"] = deg2rad(lidar_configuration["maximal_azimut_angle"])
        parameters["range_min"] = lidar_configuration["minimal_range"]
        parameters["range_max"] = lidar_configuration["maximal_range"]
        if "lms1" in lidar_configuration["model"]:
            parameters["scanner_type"] = "sick_lms_1xx"
        if "tim5" in lidar_configuration["model"]:
            parameters["scanner_type"] = "sick_tim_5xx"
        if "mrs1" in lidar_configuration["model"]:
            parameters["scanner_type"] = "sick_mrs_1xxx"
    else:
        # TODO (add other drivers)
        pass

    return parameters


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
        meta_description.get_configuration(),
        meta_description.get_geometry(),
        ros_namespace,
    )
