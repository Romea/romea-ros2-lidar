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
from ament_index_python.packages import get_package_share_directory


def get_sick_model_family(model):
    if "lms1" in model:
        return "lms1xx"
    elif "tim5" in model:
        return "tim5xx"
    elif "mrs1" in model:
        return "mrs1xxx"
    else:
        raise RuntimeError(
            "Sick "
            + model
            + " lidar is unsuported by romea_lidar_description package."
            + "Please check your configuration or contribute to support this sensor."
        )


def get_lidar_family(type, model):
    if type == "sick":
        return get_sick_model_family(model)
    else:
        raise RuntimeError(
            "No"
            + type
            + " lidar is unsuported by romea_lidar_description package."
            + "Please check your configuration or contribute to support this kind of sensor."
        )


def get_lidar_config_file(type, model, what):

    return (
        get_package_share_directory("romea_lidar_description")
        + "/config/"
        + type
        + "_"
        + get_lidar_family(type, model)
        + "_"
        + what
        + ".yaml"
    )


def get_lidar_config(type, model, what):
    with open(get_lidar_config_file(type, model, what)) as f:
        return yaml.safe_load(f)


def sick_lms1xx_configuration(model, rate, resolution):

    specifications = get_lidar_config("sick", model, "specifications")

    if not rate and not resolution:
        raise ValueError("Rate or resolution must be defined in lidar configuration")

    if rate is not None:

        if rate in specifications["rate"]:
            rate_hz = str(rate) + "hz"
        else:
            raise ValueError("Rate " + str(rate) + " is not available for lms1xx lidar")

        if resolution is not None:

            if resolution != specifications["azimut_angle_increment"][rate_hz]:
                raise ValueError(
                    "Rate "
                    + str(rate)
                    + " + resolution "
                    + str(resolution)
                    + " configuration is not available for lms1xx lidar"
                )

    else:

        if resolution in specifications["azimut_angle_increment"].values():
            rate = dict.keys()[dict.values().index(resolution)]
            rate_hz = str(rate) + "hz"
        else:
            raise ValueError(
                "Resolution " + str(resolution) + " is not available for lms1xx lidar"
            )
    sub_model = model[0:5] + "x"

    return {
        "type": specifications["type"],
        "minimal_azimut_angle": specifications["minimal_azimut_angle"],
        "maximal_azimut_angle": specifications["maximal_azimut_angle"],
        "azimut_angle_increment": specifications["azimut_angle_increment"][rate_hz],
        "azimut_angle_std": specifications["azimut_angle_std"],
        "minimal_range": specifications["minimal_range"],
        "maximal_range": specifications["maximal_range"][sub_model],
        "range_std": specifications["range_std"],
        "samples": specifications["samples"][rate_hz],
        "rate": rate,
    }


def sick_tim5xx_configuration(model, rate, resolution):

    specifications = get_lidar_config("sick", model, "specifications")

    if rate is not None and rate != specifications["rate"]:
        raise ValueError("Rate " + str(rate) + " is not available for tim5xx lidar")

    if (
        resolution is not None
        and resolution != specifications["azimut_angle_increment"][model]
    ):
        raise ValueError(
            "Resolution " + str(resolution) + " is not available for tim5xx lidar"
        )

    return {
        "type": specifications["type"],
        "minimal_azimut_angle": specifications["minimal_azimut_angle"],
        "maximal_azimut_angle": specifications["maximal_azimut_angle"],
        "azimut_angle_increment": specifications["azimut_angle_increment"][model],
        "azimut_angle_std": specifications["azimut_angle_std"],
        "minimal_range": specifications["minimal_range"],
        "maximal_range": specifications["maximal_range"][model],
        "range_std": specifications["range_std"],
        "samples": specifications["samples"][model],
        "rate": rate,
    }


def sick_mrs1xxx_configuration(model, rate, resolution):

    specifications = get_lidar_config("sick", model, "specifications")

    if rate is not None and rate != specifications["rate"]:
        raise ValueError("Rate " + str(rate) + " is not available for mrs1xxx lidar")

    if (
        resolution is not None
        and resolution not in specifications["azimut_angle_increment"]
    ):
        raise ValueError(
            "Resolution " + str(resolution) + " is not available for mrs1xxx lidar"
        )

    index = specifications["azimut_angle_increment"].index(resolution)

    return {
        "type": specifications["type"],
        "minimal_azimut_angle": specifications["minimal_azimut_angle"],
        "maximal_azimut_angle": specifications["maximal_azimut_angle"],
        "azimut_angle_increment": specifications["azimut_angle_increment"][index],
        "azimut_angle_std": specifications["azimut_angle_std"],
        "samples": specifications["samples"][index],
        "minimal_elevation_angle": specifications["minimal_elevation_angle"],
        "maximal_elevation_angle": specifications["maximal_elevation_angle"],
        "elevation_angle_increment": specifications["elevation_angle_increment"],
        "elevation_angle_std": specifications["elevation_angle_std"],
        "lasers": specifications["lasers"],
        "minimal_range": specifications["minimal_range"],
        "maximal_range": specifications["maximal_range"],
        "range_std": specifications["range_std"],
        "rate": rate,
    }


def extract_lidar_configuration(type, model, rate, resolution):
    family = get_lidar_family(type, model)
    return globals()[type + "_" + family + "_configuration"](model, rate, resolution)


def save_lidar_configuration(prefix, lidar_name, configuration):
    configuration_file_path = (
        "/tmp/"
        + prefix
        + lidar_name
        + "_configuration.yaml"
    )

    with open(configuration_file_path, "w") as f:
        yaml.dump(configuration, f)

    return configuration_file_path


def urdf(prefix, mode, name, type, model, rate, resolution,
         parent_link, xyz, rpy, ros_namespace):

    lidar_config = extract_lidar_configuration(type, model, rate, resolution)
    lidar_config_yaml_file = save_lidar_configuration(prefix, name, lidar_config)
    geometry_config_yaml_file = get_lidar_config_file(type, model, "geometry")

    xacro_file = (
        get_package_share_directory("romea_lidar_description")
        + "/urdf/lidar"+lidar_config["type"]+".xacro.urdf"
    )

    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            "prefix": prefix,
            "mode": prefix,
            "name": name,
            "sensor_config_yaml_file": lidar_config_yaml_file,
            "geometry_config_yaml_file": geometry_config_yaml_file,
            "parent_link": parent_link,
            "xyz": " ".join(map(str, xyz)),
            "rpy": " ".join(map(str, rpy)),
            "mesh_visual": str(True),
            "ros_namespace": ros_namespace
        },
    )

    return urdf_xml.toprettyxml()
