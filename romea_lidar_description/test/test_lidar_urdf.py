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
import xml.etree.ElementTree as ET
from romea_lidar_description import urdf


@pytest.fixture(scope="module")
def urdf_xml():
    prefix = "robot_"
    mode = "simulation"
    name = "lidar"

    description = {
        "type": "sick",
        "model": "lms151",
        "rate": 25,
    }

    location = {
        "parent_link": "base_link",
        "xyz": [1.0, 2.0, 3.0],
        "rpy": [4.0, 5.0, 6.0],
    }

    ros_namespace = "ns"

    with open('/tmp/urdf', 'w') as file:
        file.write(urdf(prefix, mode, name, description, location, ros_namespace))

    return ET.fromstring(urdf(prefix, mode, name, description, location, ros_namespace))


def test_lidar_name(urdf_xml):
    assert urdf_xml.find("link").get("name") == "robot_lidar_link"


def test_lidar_position(urdf_xml):
    assert urdf_xml.find("joint/origin").get("xyz") == "1.0 2.0 3.0"


def test_lidar_orientation(urdf_xml):
    print("orientation", urdf_xml.find("joint/origin").get("rpy"))
    assert (
        urdf_xml.find("joint/origin").get("rpy")
        == "0.06981317007977318 0.08726646259971647 0.10471975511965977"
    )


def test_lidar_parent_link(urdf_xml):
    assert urdf_xml.find("joint/parent").get("link") == "robot_base_link"


def test_gazebo_update_rate(urdf_xml):
    assert urdf_xml.find("gazebo/sensor/update_rate").text == "25"


def test_gazebo_horizontal_samples(urdf_xml):
    assert urdf_xml.find("gazebo/sensor/ray/scan/horizontal/samples").text == "1081"


def test_gazebo_horizontal_min_angle(urdf_xml):
    assert (
        urdf_xml.find("gazebo/sensor/ray/scan/horizontal/min_angle").text
        == "-2.356194490192345"
    )


def test_gazebo_horizontal_max_angle(urdf_xml):
    assert (
        urdf_xml.find("gazebo/sensor/ray/scan/horizontal/max_angle").text
        == "2.356194490192345"
    )


def test_gazebo_minimal_range(urdf_xml):
    assert urdf_xml.find("gazebo/sensor/ray/range/min").text == "0.05"


def test_gazebo_maximal_range(urdf_xml):
    assert urdf_xml.find("gazebo/sensor/ray/range/max").text == "50.0"


def test_plugin_namespace(urdf_xml):
    assert urdf_xml.find("gazebo/sensor/plugin/ros/namespace").text == "ns"
