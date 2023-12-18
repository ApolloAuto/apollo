#!/usr/bin/env python3

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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
config navigation mode
"""
import sys
import configparser
from modules.common_msgs.dreamview_msgs import hmi_config_pb2
from modules.common_msgs.planning_msgs import planning_config_pb2
from modules.tools.common import proto_utils

DEFAULT_NAVI_CONFIG_FILE = "/apollo/modules/tools/navigation/config/default.ini"
HMI_CONF_FILE = "/apollo/modules/dreamview/conf/hmi.conf"
PLANNING_CONF_FILE = "/apollo/modules/planning/planning_component/conf/planning_config_navi.pb.txt"
GLOBAL_FLAG_FILE = "/apollo/modules/common/data/global_flagfile.txt"
LOCALIZATION_FLAG_FILE = "/apollo/modules/localization/conf/localization.conf"
PLANNING_FLAG_FILE1 = "/apollo/modules/planning/planning_component/conf/planning.conf"
PLANNING_FLAG_FILE2 = "/apollo/modules/planning/planning_component/conf/planning_navi.conf"


def set_hmi_conf(config):
    """change hmi conf file based on navi config file"""
    hmi_conf = hmi_config_pb2.HMIConfig()
    proto_utils.get_pb_from_file(HMI_CONF_FILE, hmi_conf)

    perception = config.get('PerceptionConf', 'perception')
    navi_mode = hmi_conf.modes["Navigation"]

    if 'navigation_camera' in navi_mode.live_modules:
        navi_mode.live_modules.remove('navigation_camera')
    if 'navigation_perception' in navi_mode.live_modules:
        navi_mode.live_modules.remove('navigation_perception')

    if 'mobileye' in navi_mode.live_modules:
        navi_mode.live_modules.remove('mobileye')
    if 'third_party_perception' in navi_mode.live_modules:
        navi_mode.live_modules.remove('third_party_perception')

    if 'velodyne' in navi_mode.live_modules:
        navi_mode.live_modules.remove('velodyne')
    if 'perception' in navi_mode.live_modules:
        navi_mode.live_modules.remove('perception')

    if perception == "CAMERA":
        if 'navigation_camera' not in navi_mode.live_modules:
            navi_mode.live_modules.insert(0, 'navigation_camera')
        if 'navigation_perception' not in navi_mode.live_modules:
            navi_mode.live_modules.insert(0, 'navigation_perception')

    if perception == "MOBILEYE":
        if 'mobileye' not in navi_mode.live_modules:
            navi_mode.live_modules.insert(0, 'mobileye')
        if 'third_party_perception' not in navi_mode.live_modules:
            navi_mode.live_modules.insert(0, 'third_party_perception')

    if perception == "VELODYNE64":
        if 'velodyne' not in navi_mode.live_modules:
            navi_mode.live_modules.insert(0, 'velodyne')
        if 'perception' not in navi_mode.live_modules:
            navi_mode.live_modules.insert(0, 'perception')

    hmi_conf.modes["Navigation"].CopyFrom(navi_mode)
    proto_utils.write_pb_to_text_file(hmi_conf, HMI_CONF_FILE)


def set_planning_conf(config):
    """change planning config based on navi config"""
    planning_conf = planning_config_pb2.PlanningConfig()
    proto_utils.get_pb_from_file(PLANNING_CONF_FILE, planning_conf)
    planner_type = config.get('PlanningConf', 'planner_type')
    if planner_type == "EM":
        planning_conf.planner_type = planning_config_pb2.PlanningConfig.EM
    if planner_type == "LATTICE":
        planning_conf.planner_type = planning_config_pb2.PlanningConfig.LATTICE
    if planner_type == "NAVI":
        planning_conf.planner_type = planning_config_pb2.PlanningConfig.NAVI
    proto_utils.write_pb_to_text_file(planning_conf, PLANNING_CONF_FILE)


def set_global_flag(config):
    """update global flag file"""
    utm_zone = config.get('LocalizationConf', 'utm_zone')
    with open(GLOBAL_FLAG_FILE, 'a') as f:
        f.write('\n')
        f.write('--use_navigation_mode=true\n\n')
        f.write('--local_utm_zone_id=' + utm_zone + '\n\n')


def set_localization_flag(config):
    """update localization flag file"""
    utm_zone = config.get('LocalizationConf', 'utm_zone')
    with open(LOCALIZATION_FLAG_FILE, 'a') as f:
        f.write('\n')
        f.write('--local_utm_zone_id=' + utm_zone + '\n\n')


def set_planning_flag(config):
    """update planning flag files"""
    speed_limit = config.get('PlanningConf', 'speed_limit')
    with open(PLANNING_FLAG_FILE1, 'a') as f:
        f.write('\n')
        f.write('--planning_upper_speed_limit=' + speed_limit + '\n\n')
    with open(PLANNING_FLAG_FILE2, 'a') as f:
        f.write('\n')
        f.write('--planning_upper_speed_limit=' + speed_limit + '\n\n')


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("\nusage: python navi_config.py config.ini\n\n")
        sys.exit(0)
    config_file = sys.argv[1]
    config = configparser.ConfigParser()
    config.read(config_file)

    set_hmi_conf(config)
    set_planning_conf(config)
    set_global_flag(config)
    set_localization_flag(config)
    set_planning_flag(config)
