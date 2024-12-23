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
    assert get_lidar_specifications("sick", "lms151")['maximal_range']['lms15x'] == 50.0


def test_get_lidar_geometry_file_path_ok():
    assert (
        get_lidar_geometry_file_path("sick", "lms151")
        == get_package_share_directory("romea_lidar_description")
        + "/config/sick_lms1xx_geometry.yaml"
    )


def test_get_lidar_geometry_ok():
    assert get_lidar_geometry("sick", "lms151")['mass'] == 1.1


def test_get_lidar_complete_configuration_failed_when_rate_and_resolution_are_equal_to_none():
    with pytest.raises(ValueError) as excinfo:
        get_lidar_complete_configuration("sick", "lms151", None, None)
    assert 'Rate or resolution must be defined in lidar configuration' == str(excinfo.value)


def test_get_lidar_complete_configuration_failed_when_rate_is_wrong():
    with pytest.raises(ValueError) as excinfo:
        get_lidar_complete_configuration("sick", "lms151", 33, None)
    assert 'A 33Hz rate is not available for lms1xx lidar' == str(excinfo.value)


def test_get_lidar_complete_configuration_failed_when_resolution_is_wrong():
    with pytest.raises(ValueError) as excinfo:
        get_lidar_complete_configuration("sick", "lms151", None, 1.0)
    assert 'A 1.0° resolution is not available for lms1xx lidar' == str(excinfo.value)


def test_get_lidar_complete_configuration_failed_when_rate_and_resolution_are_not_compatible():
    with pytest.raises(ValueError) as excinfo:
        get_lidar_complete_configuration("sick", "lms151", 25, 0.5)
    assert 'A 25Hz rate and a 0.5° resolution is not an available configuration for lms1xx lidar' == str(excinfo.value)


def test_get_lidar_complete_configuration_ok():
    configuration = get_lidar_complete_configuration("sick", "lms151", 25, 0.25)
    assert configuration["type"] == "2D"
    assert configuration["maximal_range"] == 50.0
    assert configuration["azimut_angle_increment"] == 0.25
    assert configuration["samples"] == 1081
    assert configuration["rate"] == 25
