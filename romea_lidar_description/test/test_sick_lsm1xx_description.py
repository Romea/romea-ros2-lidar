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
        get_lidar_specifications_file_path("sick", "lms151")
        == get_package_share_directory("romea_lidar_description")
        + "/config/sick_lms1xx_specifications.yaml"
    )


def test_get_lidar_specifications_ok():
    assert get_lidar_specifications("sick", "lms151")['maximal_range']['dict']['lms15x'] == 50.0


def test_get_lidar_geometry_file_path_ok():
    assert (
        get_lidar_geometry_file_path("sick", "lms151")
        == get_package_share_directory("romea_lidar_description")
        + "/config/sick_lms1xx_geometry.yaml"
    )


def test_get_lidar_geometry_ok():
    assert get_lidar_geometry("sick", "lms151")['mass'] == 1.1


def test_get_lidar_complete_configuration_ok():
    user_description = {
        "type": "sick",
        "model": "lms151",
        "rate": 25,
    }

    lidar_configuration = get_lidar_complete_configuration("lidar", user_description)
    assert lidar_configuration["maximal_range"] == 50.0
    assert lidar_configuration["azimut_resolution"] == 0.25
    assert lidar_configuration["samples"] == 1081
    assert lidar_configuration["rate"] == 25


def test_get_lidar_complete_configuration_failed_when_rate_is_wrong():
    user_description = {
        "type": "sick",
        "model": "lms151",
        "rate": 33,
    }

    with pytest.raises(ValueError) as excinfo:
        get_lidar_complete_configuration("lidar", user_description)
    msg = (
        "rate value (33Hz) provided by user is not available for sick lms151 lidar called lidar, "
        + "it must be one of these values: [25, 50]"
    )
    assert msg == str(excinfo.value)


def test_get_lidar_complete_configuration_failed_when_resolution_is_wrong():

    user_description = {
        "type": "sick",
        "model": "lms151",
        "rate": 25,
        "azimut_resolution": 1.0,
    }

    with pytest.raises(ValueError) as excinfo:
        get_lidar_complete_configuration("lidar", user_description)
    msg = (
        "azimut_resolution value (1.0°) provided by user is not available "
        + "for this configuration of sick lms151 lidar called lidar, it must be equal to 0.25"
    )
    assert msg == str(excinfo.value)
