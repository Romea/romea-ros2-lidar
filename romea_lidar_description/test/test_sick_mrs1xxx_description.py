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
        get_lidar_specifications_file_path("sick", "mrs1000")
        == get_package_share_directory("romea_lidar_description")
        + "/config/sick_mrs1xxx_specifications.yaml"
    )


def test_get_lidar_specifications_ok():
    assert get_lidar_specifications("sick", "mrs1000")['samples']['dict'][0.25] == 1081


def test_get_lidar_geometry_file_path_ok():
    assert (
        get_lidar_geometry_file_path("sick", "mrs1000")
        == get_package_share_directory("romea_lidar_description")
        + "/config/sick_mrs1xxx_geometry.yaml"
    )


def test_get_lidar_geometry_ok():
    assert get_lidar_geometry("sick", "mrs1000")['mass'] == 1.2


def test_get_lidar_complete_configuration_failed_when_rate_is_wrong():
    user_description = {
        "type": "sick",
        "model": "mrs1000",
        "rate": 25,
    }
    with pytest.raises(ValueError) as excinfo:
        get_lidar_complete_configuration("lidar", user_description)
    msg = (
        "rate value (25Hz) provided by user is not available for sick mrs1000 " 
        + "lidar called lidar, it must be equal to 50"
    )
    assert msg == str(excinfo.value)


def test_get_lidar_complete_configuration_failed_when_resolution_is_wrong():
    user_description = {
        "type": "sick",
        "model": "mrs1000",
        "rate": 50,
        "azimut_resolution": 0.5,
    }

    with pytest.raises(ValueError) as excinfo:
        get_lidar_complete_configuration("lidar", user_description)
    msg = (
        "azimut_resolution value (0.5°) provided by user is not available for "
        + "sick mrs1000 lidar called lidar, it must be one of these values: [0.25, 0.125, 0.0625]"
    )
    assert msg == str(excinfo.value)


def test_get_lidar_complete_configuration_ok():
    user_description = {
        "type": "sick",
        "model": "mrs1000",
        "rate": 50,
        "azimut_resolution": 0.25,
    }

    configuration = get_lidar_complete_configuration("lidar", user_description)

    assert configuration["maximal_range"] == 64.0
    assert configuration["azimut_resolution"] == 0.25
    assert configuration["samples"] == 1081
    assert configuration["rate"] == 50
