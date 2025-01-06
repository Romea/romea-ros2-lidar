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

import math
import pytest
from ament_index_python.packages import get_package_share_directory


from romea_lidar_description import (
    get_lidar_complete_configuration,
    get_lidar_geometry_file_path,
    get_lidar_geometry,
    get_lidar_specifications_file_path,
    get_lidar_specifications,
)


def test_get_lidar_specifications_file_path_ok():
    assert (
        get_lidar_specifications_file_path("ouster", "os1_32")
        == get_package_share_directory("romea_lidar_description")
        + "/config/ouster_os1_specifications.yaml"
    )


def test_get_lidar_specifications_ok():
    assert get_lidar_specifications("ouster", "os1_32")['samples']['dict'][0.703125] == 512


def test_get_lidar_geometry_file_path_ok():
    assert (
        get_lidar_geometry_file_path("ouster", "os1_32")
        == get_package_share_directory("romea_lidar_description")
        + "/config/ouster_os1_geometry.yaml"
    )


def test_get_lidar_geometry_ok():
    assert get_lidar_geometry("ouster", "os1_32")['mass'] == 0.447


def test_get_lidar_complete_configuration_failed_when_rate_is_wrong():
    user_description = {
       "type": "ouster",
       "model": "os1_32",
       "rate": 25,
    }

    with pytest.raises(ValueError) as excinfo:
        get_lidar_complete_configuration("lidar", user_description)
    msg = (
        "rate value (25Hz) provided by user is not available for ouster os1_32 lidar "
        + "called lidar, it must be one of these values: [10, 20]"

    )

    assert msg == str(excinfo.value)


def test_get_lidar_complete_configuration_failed_when_resolution_is_wrong():

    user_description = {
       "type": "ouster",
       "model": "os1_32",
       "rate": 10,
       "azimut_angle_increment": 1.0,
    }

    with pytest.raises(ValueError) as excinfo:
        get_lidar_complete_configuration("lidar", user_description)
    msg = (
        "azimut_angle_increment value (1.0°) provided by user is not available "
        + "for ouster os1_32 lidar called lidar, it must be one of these values: "
        + "[0.703125, 0.3515625, 0.17578125]"
    )

    assert msg == str(excinfo.value)


def test_get_lidar_complete_configuration_ok():
    user_description = {
       "type": "ouster",
       "model": "os1_32",
       "rate": 10,
       "azimut_angle_increment": 0.703125,
    }

    configuration = get_lidar_complete_configuration("lidar", user_description)

    assert configuration["maximal_range"] == 55.0
    assert configuration["azimut_angle_increment"] == 0.703125 / 180 * math.pi
    assert configuration["samples"] == 512
    assert configuration["lasers"] == 32
    assert configuration["rate"] == 10
