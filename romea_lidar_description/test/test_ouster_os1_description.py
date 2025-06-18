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


from romea_lidar_description import (
    get_lidar_complete_configuration,
    get_lidar_geometry_file_path,
    get_lidar_geometry,
    get_lidar_specifications_file_path,
    get_lidar_specifications,
)

import pytest


@pytest.fixture(scope="module")
def user_description():

    return {
        "manufacturer": "ouster",
        "model": "os",
        "version": "1",
        "rate": 10
    }


def test_get_lidar_specifications_file_path_ok(user_description):
    assert (
        get_lidar_specifications_file_path(user_description)
        == get_package_share_directory("romea_lidar_description")
        + "/config/ouster_os_1_specifications.yaml"
    )


def test_get_lidar_specifications_ok(user_description):
    assert get_lidar_specifications(user_description)['samples']['list'] == [512, 1024, 2048]


def test_get_lidar_geometry_file_path_ok(user_description):
    assert (
        get_lidar_geometry_file_path(user_description)
        == get_package_share_directory("romea_lidar_description")
        + "/config/ouster_os_1_geometry.yaml"
    )


def test_get_lidar_geometry_ok(user_description):
    assert get_lidar_geometry(user_description)['mass'] == 0.447


def test_get_lidar_complete_configuration_failed_when_rate_is_wrong():
    user_description = {
       "manufacturer": "ouster",
       "model": "os",
       "version": "1",
       "rate": 25,
    }

    with pytest.raises(ValueError) as excinfo:
        get_lidar_complete_configuration("lidar", user_description)
    msg = (
        "rate value (25Hz) provided by user is not available for ouster os 1 lidar "
        + "called lidar, it must be one of these values: [10, 20]"

    )

    assert msg == str(excinfo.value)


def test_get_lidar_complete_configuration_failed_when_resolution_is_wrong():

    user_description = {
       "manufacturer": "ouster",
       "model": "os",
       "version": "1",
       "rate": 10,
       "samples": 1000,
    }

    with pytest.raises(ValueError) as excinfo:
        get_lidar_complete_configuration("lidar", user_description)
    msg = (
        "samples value (1000) provided by user is not available "
        + "for ouster os 1 lidar called lidar, it must be one of these values: [512, 1024, 2048]"
    )

    assert msg == str(excinfo.value)


def test_get_lidar_complete_configuration_ok(user_description):

    configuration = get_lidar_complete_configuration("lidar", user_description)

    assert configuration["maximal_range"] == 55.0
    assert configuration["azimut_resolution"] == 0.3515625
    assert configuration["samples"] == 1024
    assert configuration["lasers"] == 32
    assert configuration["rate"] == 10
