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
from romea_lidar_meta_bringup import LIDARMetaDescription, get_complete_driver_parameters


def test_serial_node_driver_parameters():

    meta_description_file_path = os.path.join(
        os.getcwd(), "test_lidar_sick_generic_caller_parameters.yaml"
    )
    meta_description = LIDARMetaDescription(meta_description_file_path)

    parameters = get_complete_driver_parameters(meta_description, "robot")
    assert parameters["frame_id"] == "robot_lidar_link"
    assert parameters["framerate"] == 50
    assert parameters["min_ang"] == -2.356194490192345
    assert parameters["max_ang"] == 2.356194490192345
    assert parameters["range_min"] == 0.05
    assert parameters["range_max"] == 50.0
    assert parameters["scanner_type"] == "sick_lms_1xx"
