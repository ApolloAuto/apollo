#!/usr/bin/env python3

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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
This program can dump tf2 stats

usage:
   tf_stats -f record_file
"""

import argparse

from cyber_py3.record import RecordReader
from modules.transform.proto import transform_pb2


g_args = None


def tf_stats(in_bag):
    """
    """
    reader = RecordReader(in_bag)
    global g_args
    stats = {}

    for channel, message, _type, _timestamp in reader.read_messages():
        if channel != '/tf':
            continue
        tf_pb = transform_pb2.TransformStampeds()
        tf_pb.ParseFromString(message)
        for transform in tf_pb.transforms:
            key = transform.header.frame_id + "=>" + transform.child_frame_id
            if key in stats.keys():
                stats[key] += 1
            else:
                stats[key] = 1
    print('tf stats: {}'.format(stats))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="A tool to print tf stats")
    parser.add_argument(
        "-f", action="store", type=str, help="the input data file")
    g_args = parser.parse_args()

    tf_stats(g_args.f)
