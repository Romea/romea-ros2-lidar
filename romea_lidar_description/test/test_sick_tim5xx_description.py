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
        get_lidar_specifications_file_path("sick", "tim551")
        == get_package_share_directory("romea_lidar_description")
        + "/config/sick_tim5xx_specifications.yaml"
    )


def test_get_lidar_specifications_ok():
    assert get_lidar_specifications("sick", "tim551")['maximal_range']['dict']['tim551'] == 10.0


def test_get_lidar_geometry_file_path_ok():
    assert (
        get_lidar_geometry_file_path("sick", "tim551")
        == get_package_share_directory("romea_lidar_description")
        + "/config/sick_tim5xx_geometry.yaml"
    )


def test_get_lidar_geometry_ok():
    assert get_lidar_geometry("sick", "tim551")['mass'] == 0.250


def test_get_lidar_complete_configuration_failed_when_rate_is_wrong():
    with pytest.raises(ValueError) as excinfo:
        get_lidar_complete_configuration("sick", "tim551", 25, None)
    msg = (
        "rate value (25Hz) provided by user is not available for sick tim551 lidar, "
        + "it must be equal to 15"
    )
    assert msg == str(excinfo.value)


def test_get_lidar_complete_configuration_failed_when_resolution_is_wrong():
    with pytest.raises(ValueError) as excinfo:
        get_lidar_complete_configuration("sick", "tim551", None, 0.25)
    msg = (
        "azimut_angle_increment value (0.25°) provided by user is not available "
        + "for this configuration of sick tim551 lidar, it must be equal to 1.0"
    )
    assert msg == str(excinfo.value)


def test_get_lidar_complete_configuration_ok():
    configuration = get_lidar_complete_configuration("sick", "tim551", 15, 1.0)
    assert configuration["type"] == "2D"
    assert configuration["maximal_range"] == 10.0
    assert configuration["azimut_angle_increment"] == 1.0
    assert configuration["samples"] == 271
    assert configuration["rate"] == 15
