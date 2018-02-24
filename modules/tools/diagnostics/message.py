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
Message Handle
"""
import curses

import rospy

from modules.canbus.proto import chassis_detail_pb2
from modules.canbus.proto import chassis_pb2
from modules.common.configs.proto import vehicle_config_pb2
from modules.common.monitor_log.proto import monitor_log_pb2
from modules.common.proto import drive_event_pb2
from modules.common.proto import geometry_pb2
from modules.common.proto import header_pb2
from modules.control.proto import control_cmd_pb2
from modules.control.proto import pad_msg_pb2
from modules.localization.proto import gps_pb2
from modules.localization.proto import imu_pb2
from modules.localization.proto import localization_pb2
from modules.perception.proto import perception_obstacle_pb2
from modules.perception.proto import traffic_light_detection_pb2
from modules.planning.proto import planning_internal_pb2
from modules.planning.proto import planning_pb2
from modules.prediction.proto import prediction_obstacle_pb2
from modules.routing.proto import routing_pb2
from modules.drivers.proto import mobileye_pb2
from modules.drivers.proto import delphi_esr_pb2
from modules.drivers.proto import conti_radar_pb2
from modules.monitor.proto import system_status_pb2
from modules.map.relative_map.proto import navigation_pb2

Refreshrate = 16


class Message(object):
    """
    Message Class
    """

    def __init__(self, name, proto_name, topic, period, window, lock):
        self.name = name
        self.topic = topic
        self.lock = lock
        self.proto_name = proto_name[:-2]
        self.proto = eval(proto_name)
        self.period = float(period)

        self.msg_received = False
        self.msg_new = False
        self.msg_time = rospy.get_time()
        self.msg_delay = 0
        self.msg_interval = 0
        self.msg_max = 0
        self.sequence_num = 0
        self.msg_min = float("inf")
        self.field = Field(self.proto, window, self.proto.DESCRIPTOR)

        self.display_time = rospy.get_time()

    def subscribe(self):
        """Subscribe the topic in ROS."""
        return rospy.Subscriber(
            self.topic, eval(self.proto_name), self.callback, queue_size=10)

    def callback(self, data):
        """
        callback function
        """
        nowtime = rospy.get_time()
        self.proto.CopyFrom(data)
        try:
            time = self.proto.header.timestamp_sec
            sequence_num = self.proto.header.sequence_num
        except:
            # seems no header or no timestamp in this proto data
            time = 0
            sequence_num = 0

        if self.msg_received:
            seq_diff = sequence_num - self.sequence_num
            if seq_diff != 0:
                self.msg_interval = (time - self.msg_time) * 1000 / seq_diff
            else:
                self.msg_interval = (time - self.msg_time) * 1000
            if self.msg_interval > self.msg_max:
                self.msg_max = self.msg_interval
            if self.msg_interval < self.msg_min:
                self.msg_min = self.msg_interval
        self.msg_time = time
        self.sequence_num = sequence_num
        self.msg_delay = nowtime - time

        self.msg_received = True
        if self.field.show:
            nowtime = rospy.get_time()
            if (nowtime - self.display_time) > (1.0 / Refreshrate):
                with self.lock:
                    self.field.display_on_screen()
                    self.display_time = nowtime

    def key_up(self):
        """
        Keyboard Up Key
        """
        item = self.field
        while item.show is False:
            item = item.repeatedlist[item.selection][3]
        if item.selection is not None:
            item.selection = max(item.selection - 1, 0)
        item.display_on_screen()

    def key_down(self):
        """
        Keyboard Down Key
        """
        item = self.field
        while item.show is False:
            item = item.repeatedlist[item.selection][3]
        if item.selection is not None:
            item.selection = min(item.selection + 1,
                                 len(item.repeatedlist) - 1)
        item.display_on_screen()

    def key_right(self):
        """
        Keyboard Right Key
        """
        item = self.field
        while item.show is False:
            item = item.repeatedlist[item.selection][3]
        if item.selection is not None:
            item.show = False
            item = item.repeatedlist[item.selection][3]
            item.show = True
        item.display_on_screen()

    def key_left(self):
        """
        Keyboard Left Key
        """
        item = self.field
        while item.show is False:
            item = item.repeatedlist[item.selection][3]
        item.show = False
        self.field.show = True
        self.field.display_on_screen()

    def index_incr(self):
        """
        Keyboard key to increment index in repeated item
        """
        item = self.field
        while item.show is False:
            item = item.repeatedlist[item.selection][3]
        if item.index is not None:
            item.index = min(item.index + 1, len(item.item) - 1)
        item.display_on_screen()

    def index_decr(self):
        """
        Keyboard key to decrement index in repeated item
        """
        item = self.field
        while item.show is False:
            item = item.repeatedlist[item.selection][3]
        if item.index is not None:
            item.index = max(item.index - 1, 0)
        item.display_on_screen()

    def index_begin(self):
        """
        Keyboard key to go to first element in repeated item
        """
        item = self.field
        while item.show is False:
            item = item.repeatedlist[item.selection][3]
        if item.index is not None:
            item.index = 0
        item.display_on_screen()

    def index_end(self):
        """
        Keyboard key to go to last element in repeated item
        """
        item = self.field
        while item.show is False:
            item = item.repeatedlist[item.selection][3]
        if item.index is not None:
            item.index = len(item.item) - 1
        item.display_on_screen()


class Field(object):
    """
    Item in Message Class
    """

    def __init__(self, item, window, descriptor):
        self.repeatedlist = []
        self.item = item
        self.window = window
        self.show = False
        self.selection = None
        self.descriptor = descriptor
        self.index = None
        if descriptor.containing_type is not None and \
           descriptor.label == descriptor.LABEL_REPEATED:
            if len(item) != 0:
                self.index = 0

    def display_on_screen(self):
        """
        Display Wrapper
        """
        self.window.clear()
        self.windowy, self.windowx = self.window.getmaxyx()
        self.repeatedlist = []
        if self.descriptor.containing_type is not None and \
            self.descriptor.label == self.descriptor.LABEL_REPEATED:
            if self.index is not None:
                if 'keys' in dir(self.item):
                    # For map field.
                    key = sorted(self.item.keys())[self.index]
                else:
                    key = self.index
                self.window.addstr(
                    0, 0, self.descriptor.name + ": " + str(key), curses.A_BOLD)
                self.print_out(self.item[key], self.descriptor, 1, 2)
            else:
                self.window.addstr(0, 0, self.descriptor.name + ": Empty",
                                   curses.A_BOLD)
        else:
            self.window.addstr(0, 0, self.descriptor.name, curses.A_BOLD)
            self.print_out(self.item, self.descriptor, 1, 2)

        self.print_repeated()
        self.window.refresh()

    def print_out(self, entity, descriptor, row, col):
        """
        Handles display of each item in proto
        """
        if descriptor.containing_type is None  or \
           descriptor.type == descriptor.TYPE_MESSAGE:
            for descript, item in entity.ListFields():
                if row >= self.windowy:
                    if col >= (self.windowx / 3) * 2:
                        return row, col
                    row = 0
                    col += self.windowx / 3
                if descript.label == descript.LABEL_REPEATED:
                    printstring = descript.name + ": " + str(
                        len(item)) + "[Repeated Item]"
                    repeatedlist = [
                        col, row, printstring,
                        Field(item, self.window, descript)
                    ]
                    self.repeatedlist.append(repeatedlist)
                elif descript.type == descript.TYPE_MESSAGE:
                    self.window.addstr(row, col, descript.name + ": ")
                    row, col = self.print_out(item, descript, row + 1, col + 2)
                    row -= 1
                    col -= 2
                else:
                    self.print_out(item, descript, row, col)
                row += 1
            return row, col
        elif descriptor.type == descriptor.TYPE_ENUM:
            enum_type = descriptor.enum_type.values_by_number[entity].name
            self.window.addstr(row, col, descriptor.name + ": " + enum_type)
        elif descriptor.type == descriptor.TYPE_FLOAT or descriptor.type == descriptor.TYPE_DOUBLE:
            self.window.addstr(
                row, col, descriptor.name + ": " + "{0:.5f}".format(entity))
        else:
            self.window.addnstr(row, col, descriptor.name + ": " + str(entity),
                                self.windowx / 2)
        return row, col

    def print_repeated(self):
        """
        Special Handle for displaying repeated item
        """
        indx = 0
        if (len(self.repeatedlist) != 0 and self.selection is None) or \
            self.selection >= len(self.repeatedlist):
            self.selection = 0
        if len(self.repeatedlist) == 0:
            self.selection = None
        for item in self.repeatedlist:
            if indx == self.selection:
                self.window.addstr(item[1], item[0], item[2], curses.A_REVERSE)
            else:
                self.window.addstr(item[1], item[0], item[2], curses.A_BOLD)
            indx += 1
