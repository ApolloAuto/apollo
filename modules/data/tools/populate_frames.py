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

from cyber_py import cybertron
from cyber_py import record
from modules.data.proto import frame_pb2
from modules.drivers.proto.pointcloud_pb2 import PointCloud
from modules.localization.proto.localization_pb2 import LocalizationEstimate

# Requried flags.
gflags.DEFINE_string('input_file', None, 'Input record file path.')

# Optional flags.
gflags.DEFINE_string('output_path', './', 'Output folder path.')
gflags.DEFINE_integer('maximum_frame_count', 50, 'Maximum frame count.')

# Stable flags which rarely change.
gflags.DEFINE_string('pointcloud_channel', 
                     '/apollo/sensor/velodyne64/compensator/PointCloud2',
                     'point cloud channel.')
gflags.DEFINE_string('pose_channel', 
                     '/apollo/localization/pose',
                     'point cloud channel.')
gflags.DEFINE_string('pointcloud_yaml',
                     'apollo/modules/calibration/data/'\
                     'mkz_example/velodyne_params/'\
                     'velodyne64_novatel_extrinsics_example.yaml',
                     'YAML settings for point cloud')

class FramePopulator:
    """Extract sensors data from record file, and populate to JSON."""
    def __init__(self, args):
        self._args = args
        self._current_frame_id = 0 
        self._current_frame = None
        self._frame_count = 0
        self._current_pose = frame_pb2.GPSPose() 
        self._channel_function_map = {
            args.pointcloud_channel: self.parse_protobuf_pointcloud,
            args.pose_channel: self.parse_protobuf_pose,
        }

    def load_yaml_settings(self, yaml_file_name):
        """Load settings from YAML config file."""
        yaml_file = open(yaml_file_name)
        return yaml.safe_load(yaml_file)

    def set_pointcloud_constants(self):
        """Prepare pointcloud constant settings."""
        yaml_file_name = os.path.join(apollo_root, self._args.pointcloud_yaml)
        transform_stamped = self.load_yaml_settings(yaml_file_name)
        self._current_frame.device_position.x = \
            transform_stamped['transform']['translation']['x']
        self._current_frame.device_position.y = \
            transform_stamped['transform']['translation']['y']
        self._current_frame.device_position.z = \
            transform_stamped['transform']['translation']['z']
        self._current_frame.device_heading.x = \
            transform_stamped['transform']['rotation']['x']
        self._current_frame.device_heading.y = \
            transform_stamped['transform']['rotation']['y']
        self._current_frame.device_heading.z = \
            transform_stamped['transform']['rotation']['z']
        self._current_frame.device_heading.w = \
            transform_stamped['transform']['rotation']['w']
    
    def set_cameraimage_constants(self):
        """Prepare cameraimage constant settings."""
        # TODO: pending, leave a placeholder for now
        return

    def construct_final_frame(self):
        """Construct the current frame to make it ready for dumping."""
        self.set_pointcloud_constants()
        self.set_cameraimage_constants()
        self._current_frame.device_gps_pose.CopyFrom(self._current_pose)
        self._current_frame.timestamp = self._current_frame_id / 1e9

    def dump_to_json_file(self):
        """Dump the frame content to JSON file."""
        # Construct the final frame before dumping
        self.construct_final_frame()
        self._frame_count = self._frame_count + 1
        file_name = os.path.join(self._args.output_path, 
            'frame-{}.json'.format(self._frame_count))
        jsonObj = MessageToJson(self._current_frame, False, True)
        with open(file_name, 'w') as outfile:
           outfile.write(jsonObj)

    def check_frame(self, frame_id):
        """Check if a new frame needs to be created."""
        if frame_id != self._current_frame_id:
            glog.info('#current frame {}, changing to {}'.format(
                self._current_frame_id, frame_id))
            if self._current_frame != None:
                glog.info('#dumping frame {}'.format(self._current_frame_id))
                self.dump_to_json_file()
            self._current_frame = frame_pb2.Frame()
            self._current_frame_id = frame_id

    def process(self, func, message, timestamp):
        """Process the message according to its type.""" 
        if func is not None:
            func(message, timestamp)

    def parse_protobuf_pointcloud(self, message, timestamp):
        """Process PointCloud message."""
        point_cloud = PointCloud()
        point_cloud.ParseFromString(message)

        # Use timestamp to distinguish frames
        # Convert double type of timestamp to 64bit integer
        frame_id = point_cloud.header.timestamp_sec * 1e9
        self.check_frame(frame_id)

        for pointxyzi in point_cloud.point:
            vector4 = self._current_frame.points.add()
            vector4.x = pointxyzi.x
            vector4.y = pointxyzi.y
            vector4.z = pointxyzi.z
            vector4.i = pointxyzi.intensity
    
    def parse_protobuf_pose(self, message, timestamp):
        """Process Pose message."""
        localization = LocalizationEstimate()
        localization.ParseFromString(message)
        self._current_pose.lat = localization.pose.position.x
        self._current_pose.lon = localization.pose.position.y
        self._current_pose.bearing = localization.pose.orientation.qw

    def process_record_file(self):
        """Read record file and extract the message with specified channels"""
        freader = record.RecordReader(self._args.input_file)
        time.sleep(1)
        glog.info('#processing record file {}'.format(self._args.input_file))
        for channel, message, _type, timestamp in freader.read_messages():
            if self._frame_count >= self._args.maximum_frame_count:
                glog.info('#reached the maximum frame count, exiting now')
                return
            if channel in self._channel_function_map:
                self.process(self._channel_function_map[channel], 
                    message, timestamp)

        # Dump the last frame if has not reached maximum
        if self._frame_count < self._args.maximum_frame_count:
            self.dump_to_json_file()

def main():
    """Entry point."""
    gflags.FLAGS(sys.argv)
    frame_populator = FramePopulator(gflags.FLAGS)
    frame_populator.process_record_file()
    return

if __name__ == '__main__':
    main()

