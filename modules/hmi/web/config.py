#!/usr/bin/env python

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
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
"""Global config access."""
import logging
import logging.handlers
import os

import gflags
import google.protobuf.text_format as text_format

import modules.hmi.proto.config_pb2 as config_pb2
import utils

gflags.DEFINE_string(
    'conf', 'modules/hmi/conf/config.pb.txt',
    'HMI config file, which should be text-formatted config proto.')


class Config(object):
    """Global config."""

    pb_singleton = None
    ros_root = None

    maps = {}
    vehicles = {}

    log = logging.getLogger('HMI')
    record_replay_required_modules = [
        'GPS', 'control', 'canbus', 'localization', 'dreamview', 'record_bag']

    @classmethod
    def get_pb(cls):
        """Get a pb instance from the config."""
        if cls.pb_singleton is None:
            cls.__init_once(logging.DEBUG)
        return cls.pb_singleton

    @classmethod
    def __init_once(cls, log_level=logging.DEBUG):
        """Init config once."""

        # Get ROS root.
        if os.environ.get('ROS_ROOT'):
            # ROS_ROOT env points to <ros_root>/share/ros.
            cls.ros_root = os.path.join(os.environ['ROS_ROOT'], '../..')
        else:
            # Falls back to <pwd>/ros, which is true for release docker image.
            cls.ros_root = 'ros'

        # Init the config by reading conf file.
        with open(gflags.FLAGS.conf, 'r') as conf_file:
            cls.pb_singleton = text_format.Merge(conf_file.read(),
                                                 config_pb2.Config())
        conf_pb = cls.pb_singleton
        for i in range(len(conf_pb.modules) - 1, -1, -1):
            # If the module path doesn't exist, remove it from list.
            module_path = conf_pb.modules[i].path
            if module_path and not os.path.exists(module_path):
                del conf_pb.modules[i]

        # Init logger
        file_handler = logging.handlers.TimedRotatingFileHandler(
            conf_pb.server.log_file,
            when='H', interval=1, backupCount=0)
        file_handler.setLevel(log_level)
        file_handler.setFormatter(logging.Formatter(
            '[%(name)s][%(levelname)s] %(asctime)s '
            '%(filename)s:%(lineno)s %(message)s'))
        file_handler.suffix = "%Y%m%d%H%M.log"
        cls.log.addHandler(file_handler)
        cls.log.setLevel(log_level)

        # Find available configs.
        cls.maps = utils.subdir_with_title(conf_pb.maps_dir)
        cls.vehicles = utils.subdir_with_title(conf_pb.vehicles_dir)

        cls.log.info('Get ros_root: %s', cls.ros_root)
        cls.log.info('Get config: %s', str(conf_pb))
        cls.log.info('Found maps: %s', ', '.join(cls.maps.keys()))
        cls.log.info('Found vehicles: %s', ', '.join(cls.vehicles.keys()))

    @classmethod
    def get_hardware(cls, hardware_name):
        """Get Hardware config by name."""
        return utils.find_by_name(hardware_name, cls.get_pb().hardware)

    @classmethod
    def get_module(cls, module_name):
        """Get module config by name."""
        return utils.find_by_name(module_name, cls.get_pb().modules)

    @classmethod
    def get_tool(cls, tool_name):
        """Get module config by name."""
        return utils.find_by_name(tool_name, cls.get_pb().tools)

    @classmethod
    def get_ros_path(cls, relative_path):
        """Get path which is relative to ros root."""
        return os.path.join(cls.ros_root, relative_path)
