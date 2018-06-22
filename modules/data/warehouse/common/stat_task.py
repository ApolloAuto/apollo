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
"""
Stat task.
Usage:
    ./stat_task.py <task_dir>

    <task_dir> is a generally a directory under data/bag/ which contains a
    sequence of bags recorded consequently.
"""

import collections
import glob
import os
import sys

from rosbag.bag import Bag
import gflags

from modules.canbus.proto.chassis_pb2 import Chassis
from modules.data.proto.static_info_pb2 import StaticInfo
from modules.data.proto.task_pb2 import Task
from modules.localization.proto.localization_pb2 import LocalizationEstimate
import coord_calculator

gflags.DEFINE_float('pos_sample_duration', 2, 'In seconds.')
gflags.DEFINE_float('pos_sample_min_distance', 3, 'In meters.')


class TaskCalculator(object):
    """Calculate the task and produce apollo.data.Task."""

    kChassisTopic = '/apollo/canbus/chassis'
    kLocalizationTopic = '/apollo/localization/pose'
    kStaticInfoTopic = '/apollo/monitor/static_info'
    kTopics = [
        kChassisTopic,
        kLocalizationTopic,
        kStaticInfoTopic,
    ]

    @staticmethod
    def stat(task_dir):
        """Calculate the task and produce apollo.data.Task."""
        return TaskCalculator().process_task(task_dir)


    def __init__(self):
        """Init."""
        # To output.
        self.task = Task()

        # Localization memory.
        self.last_pos = None
        self.last_pos_sampled = None
        self.last_pos_sampled_time = -1

        # DrivingMode memory.
        self.current_driving_mode = None
        # {DrivingMode: meters}
        self.mileage = collections.defaultdict(float)

        # StaticInfo memory.
        self.last_static_info = None

        self.processors = {
            self.kChassisTopic: self._on_chassis,
            self.kLocalizationTopic: self._on_localization,
            self.kStaticInfoTopic: self._on_static_info,
        }

    def process_task(self, task_dir):
        """Process a task and produce apollo.data.Task."""
        # Scan bags.
        bags = sorted(glob.glob(os.path.join(task_dir, '*.bag')))
        print 'Found {} bags'.format(len(bags))
        for bag_file in bags:
            self.process_bag(bag_file)
        if len(self.task.bags) == 0:
            sys.stderr.write('No bags were processed successfully!\n')
            self.task = None
            return None

        self.task.start_time = self.task.bags[0].start_time
        self.task.end_time = self.task.bags[-1].end_time

        # Populate static info.
        if self.last_static_info:
            # TODO(xiaoxq): They are all StaticInfo proto, but strangely can not
            # assign with CopyFrom(). Use serialize/deserialize as work around.
            self.task.info.ParseFromString(
                self.last_static_info.SerializeToString())

        # Populate ID.
        # We recommend to structure task as <vehicle_name>/<task_name>/*.bag.
        task_dir = os.path.abspath(task_dir)
        vehicle_name = self.task.info.vehicle.name.lower() or os.path.basename(
            os.path.dirname(task_dir))
        task_name = os.path.basename(task_dir)
        self.task.id = '{}/{}'.format(vehicle_name, task_name)

        # Populate topics.
        topic_set = set()
        for bag in self.task.bags:
            topic_set.update(bag.topics.keys())
        self.task.topics.extend(sorted(topic_set))

        # Populate mileage.
        for mode, meters in self.mileage.items():
            self.task.mileage[Chassis.DrivingMode.Name(mode)] = meters

        # Populate loop type.
        if Chassis.COMPLETE_AUTO_DRIVE in self.mileage:
            self.task.loop_type = Task.CLOSE_LOOP
        else:
            self.task.loop_type = Task.OPEN_LOOP
        return self.task

    def process_bag(self, bag_file):
        """Process a bag and populate task proto."""
        print 'Processing bag', bag_file
        with Bag(bag_file, 'r') as bag:
            if bag.get_message_count() == 0:
                return
            bag_pb = self.task.bags.add()
            bag_pb.name = os.path.basename(bag.filename)
            bag_pb.size = bag.size
            bag_pb.size = bag.size
            bag_pb.version = bag.version
            bag_pb.start_time = bag.get_start_time()
            bag_pb.end_time = bag.get_end_time()
            bag_pb.msg_count = bag.get_message_count()
            for topic, info in bag.get_type_and_topic_info().topics.items():
                bag_pb.topics[topic].msg_type = info.msg_type
                bag_pb.topics[topic].msg_count = info.message_count
                if info.frequency:
                    bag_pb.topics[topic].frequency = info.frequency
            # Scan messages.
            for topic, msg, t in bag.read_messages(topics=self.kTopics):
                self.processors[topic](msg, t)

    def _on_chassis(self, msg, t):
        if (self.current_driving_mode == Chassis.COMPLETE_AUTO_DRIVE and
            msg.driving_mode == Chassis.EMERGENCY_MODE):
            disengagement = self.task.disengagements.add()
            disengagement.time = t.to_sec()
            if self.last_pos:
                lat, lng = coord_calculator.utm_to_latlng(self.last_pos.x,
                                                          self.last_pos.y)
                disengagement.location.latitude = lat
                disengagement.location.longitude = lng
        self.current_driving_mode = msg.driving_mode

    def _on_localization(self, msg, t):
        new_pos = msg.pose.position

        # Update mileage and position.
        if self.last_pos and (self.current_driving_mode is not None):
            self.mileage[self.current_driving_mode] += \
                coord_calculator.utm_distance(self.last_pos, new_pos)
        self.last_pos = new_pos

        # Sample map point.
        time_sec = t.to_sec()
        should_sample = True
        if self.last_pos_sampled:
            G = gflags.FLAGS
            if time_sec - self.last_pos_sampled_time < G.pos_sample_duration:
                should_sample = False
            elif (coord_calculator.utm_distance(self.last_pos_sampled, new_pos)
                  < G.pos_sample_min_distance):
                should_sample = False
        if should_sample:
            self.last_pos_sampled = new_pos
            self.last_pos_sampled_time = time_sec
            lat, lng = coord_calculator.utm_to_latlng(new_pos.x, new_pos.y)
            map_point = self.task.map_path.add()
            map_point.latitude, map_point.longitude = lat, lng

    def _on_static_info(self, msg, t):
        # Newer StaticInfo always overwrite old one
        self.last_static_info = msg


if __name__ == '__main__':
    gflags.FLAGS(sys.argv)
    task_dir = sys.argv[-1]
    print TaskCalculator.stat(task_dir)
