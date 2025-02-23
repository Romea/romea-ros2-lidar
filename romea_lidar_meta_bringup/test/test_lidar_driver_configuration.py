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
from romea_lidar_meta_bringup import LIDARMetaDescription, get_driver_launch_file_configuration


def get_meta_description(profile_filename):
    meta_description_file_path = os.path.join(os.getcwd(), profile_filename)
    return LIDARMetaDescription(meta_description_file_path, "robot")


def get_nodes_configuration(profile_filename, mode="live"):
    meta_description = get_meta_description(profile_filename)
    return get_driver_launch_file_configuration(meta_description, mode)


def test_sick_scan_xd_lms1xx_profile():

    nodes_configuration = get_nodes_configuration("test_sick_scan_xd_lms1xx_profile.yaml")

    assert nodes_configuration["driver"]["package"] == "sick_scan_xd"
    assert nodes_configuration["driver"]["executable"] == "sick_generic_caller"
    assert nodes_configuration["driver"]["parameters"]["frame_id"] == "robot_lidar_link"
    assert nodes_configuration["driver"]["parameters"]["hostname"] == "192.168.1.112"
    assert nodes_configuration["driver"]["parameters"]["port"] == 2112
    assert nodes_configuration["driver"]["parameters"]["min_ang"] == -2.356194490192345
    assert nodes_configuration["driver"]["parameters"]["max_ang"] == 2.356194490192345
    assert nodes_configuration["driver"]["parameters"]["range_min"] == 0.05
    assert nodes_configuration["driver"]["parameters"]["range_max"] == 50.0
    assert nodes_configuration["driver"]["parameters"]["scanner_type"] == "sick_lms_1xx"
