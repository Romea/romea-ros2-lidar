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
import os
import time
import unittest
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import rclpy
from rclpy.qos import qos_profile_sensor_data
import romea_lidar_meta_bringup.meta_description as meta_description
from sensor_msgs.msg import LaserScan


def generate_test_description():
    try:
        get_package_share_directory("gazebo_ros")
        simulator_type = "gazebo_classic"
    except PackageNotFoundError:
        simulator_type = "gazebo"

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_lidar_meta_bringup")
            + "/launch/simulation_test.launch.py"
        ),
        launch_arguments={
            "gui": "false",
            "simulator_type": simulator_type,
            "robot_namespace": "robot",
            "meta_description_file_path": os.path.join(
                os.path.dirname(__file__), "test_simulation_2d.yaml"
            ),
        }.items(),
    )
    return LaunchDescription([
        simulation,
        launch_testing.actions.ReadyToTest(),
    ])


class TestLIDAR2DGazeboSimulation(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_lidar2d_gazebo_simulation")
        cls.messages = []
        cls.subscription = cls.node.create_subscription(
            LaserScan, "/robot/ns/lidar/scan", cls.messages.append, qos_profile_sensor_data
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return
        self.fail("Timed out: " + description)

    def test_lidar_world_attachment(self):
        path = os.path.join(os.path.dirname(__file__), "test_simulation_2d.yaml")
        description = meta_description.LIDARMetaDescription(path, "robot")
        urdf = ET.fromstring(meta_description.generate_xml_urdf_description_str(
            "simulation_gazebo", description, standalone=True
        ))
        self.assertIsNotNone(urdf.find("link[@name='world']"))
        joint = urdf.find("joint[@name='robot_lidar_joint']")
        self.assertEqual(joint.find("parent").get("link"), "world")
        self.assertEqual(joint.find("child").get("link"), "robot_lidar_link")
        roll, pitch, yaw = map(float, joint.find("origin").get("rpy").split())
        self.assertAlmostEqual(roll, math.radians(4))
        self.assertAlmostEqual(pitch, math.radians(5))
        self.assertAlmostEqual(yaw, math.radians(6))

    def test_lidar_publishes_data(self):
        self.wait_until(lambda: len(self.messages) >= 5, 60, "no lidar scans received")
        stamps = []
        for message in self.messages[:5]:
            self.assertEqual(message.header.frame_id, "robot_lidar_link")
            stamps.append(message.header.stamp.sec * 10**9 + message.header.stamp.nanosec)
        self.assertGreater(stamps[0], 0)
        self.assertTrue(all(current > previous for previous, current in zip(stamps, stamps[1:])))
