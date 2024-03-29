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


import os
import pytest
from numpy import deg2rad, radians

from romea_lidar_bringup import LIDARMetaDescription


@pytest.fixture(scope="module")
def meta_description():
    meta_description_file_path = os.path.join(os.getcwd(), "test_lidar_bringup.yaml")
    return LIDARMetaDescription(meta_description_file_path)


def test_get_name(meta_description):
    assert meta_description.get_name() == "lidar"


def test_get_namespace(meta_description):
    assert meta_description.get_namespace() == "ns"


def test_has_driver_configuration(meta_description):
    assert meta_description.has_driver_configuration() is True


def test_get_driver_pkg(meta_description):
    assert meta_description.get_driver_pkg() == "sick_scan"


def test_get_driver_executable(meta_description):
    assert meta_description.get_driver_executable() == "sick_generic_caller"


def test_get_driver_parameters(meta_description):
    parameters = meta_description.get_driver_parameters()
    assert parameters["hostname"] == "192.168.1.112"
    assert parameters["port"] == "2112"


def test_get_model(meta_description):
    assert meta_description.get_model() == "lms151"


def test_get_rate(meta_description):
    assert meta_description.get_rate() == 50


def test_get_resolution_deg(meta_description):
    assert meta_description.get_resolution_deg() == 0.5


def test_get_resolution_rad(meta_description):
    assert meta_description.get_resolution_rad() == deg2rad(0.5)


def test_get_parent_link(meta_description):
    assert meta_description.get_parent_link() == "base_link"


def test_get_xyz(meta_description):
    assert meta_description.get_xyz() == [1.0, 2.0, 3.0]


def test_get_rpy_deg(meta_description):
    assert meta_description.get_rpy_deg() == [4.0, 5.0, 6.0]


def test_get_rpy_rad(meta_description):
    assert meta_description.get_rpy_rad() == radians([4.0, 5.0, 6.0]).tolist()


def test_get_records(meta_description):
    records = meta_description.get_records()
    assert records["scan"] is True
    assert records["cloud"] is False
