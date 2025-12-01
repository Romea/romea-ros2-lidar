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

from ament_index_python.packages import get_package_share_directory

import pytest

from romea_lidar_description import (
    get_complete_configuration,
    get_geometry,
    get_geometry_file_path,
    get_specifications,
    get_specifications_file_path,
)


@pytest.fixture(scope="module")
def user_description():

    return {
        "manufacturer": "sick",
        "model": "tim",
        "version": "551",
        "rate": 15,
    }


def test_get_specifications_file_path_ok(user_description):
    assert (
        get_specifications_file_path(user_description)
        == get_package_share_directory("romea_lidar_description")
        + "/config/sick_tim_5xx_specifications.yaml"
    )


def test_get_specifications_ok(user_description):
    assert get_specifications(user_description)['maximal_range']['dict']['551'] == 10.0


def test_get_geometry_file_path_ok(user_description):
    assert (
        get_geometry_file_path(user_description)
        == get_package_share_directory("romea_lidar_description")
        + "/config/sick_tim_5xx_geometry.yaml"
    )


def test_get_geometry_ok(user_description):
    assert get_geometry(user_description)['mass'] == 0.250


def test_get_complete_configuration_failed_when_rate_is_wrong():
    user_description = {
        "manufacturer": "sick",
        "model": "tim",
        "version": "551",
        "rate": 25,
    }

    with pytest.raises(ValueError) as excinfo:
        get_complete_configuration("lidar", user_description, {})
    msg = (
        "rate value (25Hz) provided by user is not available for sick tim 551 lidar called lidar, "
        + "it must be equal to 15"
    )

    assert msg == str(excinfo.value)


def test_get_complete_configuration_failed_when_resolution_is_wrong():
    user_description = {
        "manufacturer": "sick",
        "model": "tim",
        "version": "551",
        "azimut_resolution": 0.25,
    }

    with pytest.raises(ValueError) as excinfo:
        get_complete_configuration("lidar", user_description, {})
    msg = (
        "azimut_resolution value (0.25°) provided by user is not available "
        + "for this configuration of sick tim 551 lidar called lidar, it must be equal to 1.0"
    )

    assert msg == str(excinfo.value)


def test_get_complete_configuration_ok(user_description):

    configuration = get_complete_configuration("lidar", user_description, {})

    assert configuration["maximal_range"] == 10.0
    assert configuration["azimut_resolution"] == 1.0
    assert configuration["samples"] == 271
    assert configuration["rate"] == 15
