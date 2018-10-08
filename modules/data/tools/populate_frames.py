#!/usr/bin/env python

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
Extract messages of specified topics from data record file,
and populate them into frames

Usage:
    populate_frames.py --input_file=a.record

See the gflags for more optional args.
"""

import os
import sys
import time

import gflags
import glog
import yaml

import frame

apollo_root = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '../../../..'))
py_proto_path = os.path.join(apollo_root, 'py_proto')
sys.path.append(py_proto_path)

from cybertron import cybertron
from cybertron import record
from modules.drivers.proto.pointcloud_pb2 import PointCloud

# Requried flags.
gflags.DEFINE_string('input_file', None, 'Input record file path.')

# Optional flags.
gflags.DEFINE_string('output_path', './', 'Output path.')

# Stable flags which rarely change.
gflags.DEFINE_string('pointcloud_channel', 
                     '/apollo/sensor/velodyne64/compensator/PointCloud2',
                     'pointcloud channel.')

class FramePopulator:
    def __init__(self, args):
        self._args = args
        self._current_frame_id = -1
        self._current_frame = None
        self._channel_function_map = {
            args.pointcloud_channel: self.parse_protobuf_pointcloud,
        }

    def load_yaml_settings(self):
        # TODO: LOAD YAML FILE AND GET ITS SETTINGS HERE
        return

    def check_frame(self, frame_id):
        if frame_id != self._current_frame_id:
            if self._current_frame != None:
                self.load_yaml_settings()
                self._current_frame.dump_to_file(self._args.output_file)
            self._current_frame = frame.Frame()
            self._current_frame_id = frame_id

    def process(self, func, message, timestamp):
        if func is not None:
            func(message, timestamp)

    def parse_protobuf_pointcloud(self, message, timestamp):
        point_cloud = PointCloud()
        point_cloud.ParseFromString(message)

        self.check_frame(point_cloud.frame_id)

        #print ('width: %d' % point_cloud.width)
        #print ('height: %d' % point_cloud.height)
        
        for pointxyzi in point_cloud.point:
            vector4 = frame.Vector4(
                pointxyzi.x,
                pointxyzi.y,
                pointxyzi.z,
                pointxyzi.i,
                pointxyzi.is_ground
            )
            self._current_frame.points.append(vector4)

    def process_record_file(self):
        """read record file and extract the message with specified channels"""
        freader = record.RecordReader(self._args.input_file)
        time.sleep(1)

        glog.info('#processing record file {}'.format(self._args.input_file))

        for channel, message, _type, timestamp in freader.read_messages():
            self.process(self._channel_function_map[channel], 
                message, timestamp)

def main():
    gflags.FLAGS(sys.argv)

    frame_populator = FramePopulator(gflags.FLAGS)
    frame_populator.process_record_file()

    return

if __name__ == '__main__':
    main()

