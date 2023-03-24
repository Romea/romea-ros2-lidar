# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

import os
import pytest
import subprocess

from romea_lidar_bringup import urdf_description
from ament_index_python import get_package_prefix
import xml.etree.ElementTree as ET


@pytest.fixture(scope="module")
def urdf():

    exe = (
        get_package_prefix("romea_lidar_bringup")
        + "/lib/romea_lidar_bringup/urdf_description.py"
    )

    meta_description_file_path = os.path.join(os.getcwd(), "test_lidar_bringup.yaml")

    print(urdf_description("robot_", meta_description_file_path))

    return ET.fromstring(
        subprocess.check_output(
            [
                exe,
                "robot_namespace:robot",
                "meta_description_file_path:" + meta_description_file_path,
            ],
            encoding="utf-8",
        )
    )


def test_lidar_name(urdf):
    assert urdf.find("link").get("name") == "robot_lidar_link"


def test_lidar_position(urdf):
    print("xyz", urdf.find("joint/origin").get("xyz"))
    assert urdf.find("joint/origin").get("xyz") == "1.0 2.0 3.0 "


def test_lidar_orientation(urdf):
    assert (
        urdf.find("joint/origin").get("rpy")
        == "0.06981317007977318 0.08726646259971647 0.10471975511965978"
    )


def test_lidar_parent_link(urdf):
    assert urdf.find("joint/parent").get("link") == "robot_base_link"


def test_lidar_rate(urdf):
    assert urdf.find("gazebo/sensor/update_rate").text == "50"


def test_plugin_namespace(urdf):
    assert urdf.find("gazebo/sensor/plugin/ros/namespace").text == "/robot/ns/lidar"
