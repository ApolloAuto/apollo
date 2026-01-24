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
Slam Template Generator
"""
import os

from modules.conf_gen.diff_gen.file_parser import FileParser
import modules.conf_gen.common.color_print as p

# LOAM 建图
LOAM_CONF_DIR = 'modules/loam_velodyne/conf'
LOAM_REGISTRATION_CONF_FILE = 'laser_multiscan_registration_conf.pb.txt'
LOAM_ODOMETRY_CONF_FILE = 'laser_odometry_conf.pb.txt'
LOAM_LOCALIZATION_CONF_FILE = 'localization_conf.pb.txt'
LOAM_LOCALIZATION_FLAGS_FILE = 'localization_conf.conf'

LOAM_INDOOR_CONF_DIR = 'modules/loam_velodyne_indoor/conf'
LOAM_INDOOR_REGISTRATION_CONF_FILE = 'laser_multiscan_registration_conf.pb.txt'
LOAM_INDOOR_ODOMETRY_CONF_FILE = 'laser_odometry_conf.pb.txt'
LOAM_INDOOR_LOCALIZATION_CONF_FILE = 'localization_conf.pb.txt'
LOAM_INDOOR_LOCALIZATION_FLAGS_FILE = 'localization_conf.conf'

# SLAM LOCAL 定位
SLAM_CONF_DIR = 'modules/slam_local/conf'
SLAM_REGISTRATION_CONF_FILE = 'laser_multiscan_registration_conf.pb.txt'
SLAM_ODOMETRY_CONF_FILE = 'laser_odometry_conf.pb.txt'
SLAM_LOCALIZATION_CONF_FILE = 'localization_conf.pb.txt'
SLAM_LOCALIZATION_FLAGS_FILE = 'slam_local.flag'

SLAM_INDOOR_CONF_DIR = 'modules/slam_local_indoor/conf'
SLAM_INDOOR_REGISTRATION_CONF_FILE = 'laser_multiscan_registration_conf.pb.txt'
SLAM_INDOOR_ODOMETRY_CONF_FILE = 'laser_odometry_conf.pb.txt'
SLAM_INDOOR_LOCALIZATION_CONF_FILE = 'localization_conf.pb.txt'
SLAM_INDOOR_LOCALIZATION_FLAGS_FILE = 'slam_local_indoor.flag'

SLAM_CONF_LIST = [
    (LOAM_CONF_DIR, {
        LOAM_REGISTRATION_CONF_FILE: ['pointCloudTopic', 'lidarFrame', 'N_SCAN'],
        LOAM_ODOMETRY_CONF_FILE: ['pointCloudTopic', 'lidarFrame', 'N_SCAN'],
        LOAM_LOCALIZATION_CONF_FILE: ['pointCloudTopic', 'lidarFrame', 'N_SCAN'],
        LOAM_LOCALIZATION_FLAGS_FILE: ['imu_gps_translation_x', 'imu_gps_translation_y',
                                       'imu_gps_translation_z', 'zoneid']
    }),
    (LOAM_INDOOR_CONF_DIR, {
        LOAM_INDOOR_REGISTRATION_CONF_FILE: ['pointCloudTopic', 'lidarFrame', 'N_SCAN'],
        LOAM_INDOOR_ODOMETRY_CONF_FILE: ['pointCloudTopic', 'lidarFrame', 'N_SCAN'],
        LOAM_INDOOR_LOCALIZATION_CONF_FILE: ['pointCloudTopic', 'lidarFrame', 'N_SCAN'],
        LOAM_INDOOR_LOCALIZATION_FLAGS_FILE: ['imu_gps_translation_x', 'imu_gps_translation_y',
                                              'imu_gps_translation_z']
    }),
    (SLAM_CONF_DIR, {
        SLAM_REGISTRATION_CONF_FILE: ['pointCloudTopic', 'lidarFrame', 'baselinkFrame', 'N_SCAN'],
        SLAM_ODOMETRY_CONF_FILE: ['pointCloudTopic', 'lidarFrame', 'N_SCAN'],
        SLAM_LOCALIZATION_CONF_FILE: ['pointCloudTopic', 'lidarFrame', 'N_SCAN'],
        SLAM_LOCALIZATION_FLAGS_FILE: ['imu_gps_translation_x', 'imu_gps_translation_y',
                                       'imu_gps_translation_z', 'zoneid']
    }),
    (SLAM_INDOOR_CONF_DIR, {
        SLAM_INDOOR_REGISTRATION_CONF_FILE: ['pointCloudTopic', 'lidarFrame', 'baselinkFrame', 'N_SCAN'],
        SLAM_INDOOR_ODOMETRY_CONF_FILE: ['pointCloudTopic', 'lidarFrame', 'N_SCAN'],
        SLAM_INDOOR_LOCALIZATION_CONF_FILE: ['pointCloudTopic', 'lidarFrame', 'N_SCAN'],
        SLAM_INDOOR_LOCALIZATION_FLAGS_FILE: ['imu_gps_translation_x', 'imu_gps_translation_y',
                                              'imu_gps_translation_z']
    })
]


class SlamTplGenerator:
    """ Slam Template Generator
    """

    def __init__(self, ENV, main_lidar, imu_ant_offset, gnss_conf):
        self.env = ENV
        self.params = {
            'pointCloudTopic': main_lidar['point_cloud_channel'],
            'lidarFrame': main_lidar['frame_id'],
            'baselinkFrame': main_lidar['frame_id'],
            'N_SCAN': main_lidar['lidar_nscan'],
            'imu_gps_translation_x': imu_ant_offset['imu_gps_translation_x'],
            'imu_gps_translation_y': imu_ant_offset['imu_gps_translation_y'],
            'imu_gps_translation_z': imu_ant_offset['imu_gps_translation_z'],
            'zoneid': gnss_conf['zone_id']
        }

    def gen(self):
        """ generate all lidar conf
        """
        if not self.gen_conf():
            p.error('Generate slam conf failed.')
            return False
        return True

    def gen_conf(self):
        """ generate lidar conf
        """
        for (dirname, files) in SLAM_CONF_LIST:
            for filename, keys in files.items():
                relative_path = os.path.join(dirname, filename)
                diff_params = {'update': [{key: self.params[key]} for key in keys]}
                file_parser = FileParser(self.env, relative_path, diff_params)
                if not file_parser.gen():
                    p.error(f'generate {relative_path} diff failed.')
                    return False
        return True
