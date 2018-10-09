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
and populate them into frames with JSON format

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
from google.protobuf.json_format import MessageToJson

apollo_root = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '../../../..'))
py_proto_path = os.path.join(apollo_root, 'py_proto')
sys.path.append(py_proto_path)

from cybertron import cybertron
from cybertron import record
from modules.data.proto import frame_pb2
from modules.drivers.proto.pointcloud_pb2 import PointCloud

# Requried flags.
gflags.DEFINE_string('input_file', None, 'Input record file path.')

# Optional flags.
gflags.DEFINE_string('output_path', './', 'Output folder path.')

# Stable flags which rarely change.
gflags.DEFINE_string('pointcloud_channel', 
                     '/apollo/sensor/velodyne64/compensator/PointCloud2',
                     'pointcloud channel.')

class FramePopulator:
    def __init__(self, args):
        self._args = args
        self._current_frame_id = '#' 
        self._current_frame = None
        self._frame_count = 0
        self._channel_function_map = {
            args.pointcloud_channel: self.parse_protobuf_pointcloud,
        }

    def load_yaml_settings(self):
        # TODO: LOAD YAML FILE AND GET ITS SETTINGS HERE
        return

    def dump_to_json_file(self):
        self._frame_count = self._frame_count + 1
        file_name = os.path.join(self._args.output_path, 
            'frame{}.json'.format(self._frame_count))
        jsonObj = MessageToJson(self._current_frame)
        with open(file_name, 'w') as outfile:
           outfile.write(jsonObj)

    def check_frame(self, frame_id):
        if frame_id != self._current_frame_id:
            glog.info('#current frame {}, changing to {}'.format(
                self._current_frame_id, frame_id))
            if self._current_frame != None:
                glog.info('#dumping frame {}'.format(self._current_frame_id))
                self.load_yaml_settings()
                self.dump_to_json_file()
            self._current_frame = frame_pb2.Frame()
            self._current_frame_id = frame_id

    def process(self, func, message, timestamp):
        if func is not None:
            func(message, timestamp)

    def parse_protobuf_pointcloud(self, message, timestamp):
        point_cloud = PointCloud()
        point_cloud.ParseFromString(message)

        self.check_frame(point_cloud.frame_id)

        for pointxyzi in point_cloud.point:
            vector4 = self._current_frame.points.add()
            vector4.x = pointxyzi.x
            vector4.y = pointxyzi.y
            vector4.z = pointxyzi.z
            vector4.i = pointxyzi.intensity

    def process_record_file(self):
        """read record file and extract the message with specified channels"""
        freader = record.RecordReader(self._args.input_file)
        time.sleep(1)

        glog.info('#processing record file {}'.format(self._args.input_file))

        for channel, message, _type, timestamp in freader.read_messages():
            if channel in self._channel_function_map:
                self.process(self._channel_function_map[channel], 
                    message, timestamp)

        self.dump_to_json_file()

def main():
    gflags.FLAGS(sys.argv)

    frame_populator = FramePopulator(gflags.FLAGS)
    frame_populator.process_record_file()

    return

if __name__ == '__main__':
    main()

