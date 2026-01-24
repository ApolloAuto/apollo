#!/usr/bin/env python3

###############################################################################
# Copyright 2024 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################
"""
Template Generator
"""
from modules.conf_gen.tpl_gen.gnss_tpl_gen import GnssTplGenerator
from modules.conf_gen.tpl_gen.camera_tpl_gen import CameraTplGenerator
from modules.conf_gen.tpl_gen.lidar_tpl_gen import LidarTplGenerator
from modules.conf_gen.tpl_gen.perception_tpl_gen import PerceptionTplGenerator
from modules.conf_gen.tpl_gen.slam_tpl_gen import SlamTplGenerator
from modules.conf_gen.tpl_gen.transform_tpl_gen import TransformTplGenerator
from modules.conf_gen.tpl_gen.vehicle_tpl_gen import VehicleTplGenerator
from modules.conf_gen.tpl_gen.tools_tpl_gen import ToolsTplGenerator
from modules.conf_gen.tpl_gen.cyber_tpl_gen import CyberTplGenerator
from modules.conf_gen.tpl_gen.safety_tpl_gen import SafetyTplGenerator


class TplGenerator:
    """ Template Generator
    """

    def __init__(self, ENV):
        self.env = ENV

    def gen(self):
        """ gen tpl file
        """
        vehicle_tpl_generator = VehicleTplGenerator(self.env)
        if not vehicle_tpl_generator.gen():
            return False
        transform_tpl_generator = TransformTplGenerator(self.env)
        if not transform_tpl_generator.gen():
            return False
        gnss_tpl_generator = GnssTplGenerator(self.env)
        if not gnss_tpl_generator.gen():
            return False
        lidar_tpl_generator = LidarTplGenerator(self.env)
        if not lidar_tpl_generator.gen():
            return False
        camera_tpl_generator = CameraTplGenerator(self.env)
        if not camera_tpl_generator.gen():
            return False
        if not lidar_tpl_generator.main_lidar:
            return True
        perception_tpl_generator = PerceptionTplGenerator(
            self.env, lidar_tpl_generator.virtual_lidar or lidar_tpl_generator.main_lidar,
            camera_tpl_generator.camera_conf_tpl)
        if not perception_tpl_generator.gen():
            return False
        slam_tpl_generator = SlamTplGenerator(self.env, lidar_tpl_generator.main_lidar,
                                              transform_tpl_generator.imu_ant_offset,
                                              gnss_tpl_generator.gnss_conf_tpl)
        if not slam_tpl_generator.gen():
            return False
        tools_conf_generator = ToolsTplGenerator(self.env,
                                                 lidar_tpl_generator.lidar_list,
                                                 transform_tpl_generator.lidar_params_tpl,
                                                 camera_tpl_generator.camera_conf_tpl)
        if not tools_conf_generator.gen():
            return False
        cyber_conf_generator = CyberTplGenerator(self.env, lidar_tpl_generator.lidar_list)
        if not cyber_conf_generator.gen():
            return False
        safety_conf_generator = SafetyTplGenerator(self.env, lidar_tpl_generator.lidar_list)
        if not safety_conf_generator.gen():
            return False
        return True
