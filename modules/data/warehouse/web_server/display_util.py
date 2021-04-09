#!/usr/bin/env python
# -*- coding: UTF-8-*-
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
"""Utils for displaying."""

import datetime
import math
import os
import pytz
import sys

import gflags

from modules.canbus.proto.chassis_pb2 import Chassis
from modules.data.proto.task_pb2 import Task

gflags.DEFINE_string('timezone', 'America/Los_Angeles', 'Timezone.')


def timestamp_to_time(timestamp):
    """Convert Unix epoch timestamp to readable time."""
    dt = datetime.datetime.fromtimestamp(timestamp, pytz.utc)
    local_tz = pytz.timezone(gflags.FLAGS.timezone)
    return dt.astimezone(local_tz).strftime('%Y-%m-%d %H:%M:%S')


def task_id_to_path(task_id):
    """Convert task id to date."""
    parts = task_id.split('/', 1)
    date_len = len('2018-01-01')
    if len(parts) < 2 or len(parts[1]) < date_len:
        return 'Null'
    task_date = parts[1][:date_len]
    return os.path.join('public_test', task_date, task_id)


def loop_type(type_value):
    """Convert loop_type to be readable."""
    if type_value == Task.OPEN_LOOP:
        return 'Open Loop'
    if type_value == Task.CLOSE_LOOP:
        return 'Close Loop'
    return ''


def find_bag_by_time(bags, timestamp):
    """Find bag name by given timestamp."""
    for bag in bags:
        if timestamp <= bag.end_time:
            return bag.name if timestamp >= bag.start_time else 'None'
    return 'Null'


def find_bag_offset_by_time(bags, timestamp):
    """Find offset of time in bag."""
    for bag in bags:
        if timestamp <= bag.end_time:
            offset = timestamp - bag.start_time
            return offset if offset >= 0 else '-1'
    return '-1'


def draw_path_on_gmap(map_path, canvas_id):
    """Draw map path on Google map."""
    if not map_path:
        return ''
    # Get center and zoom.
    min_lat, max_lat = sys.float_info.max, -sys.float_info.max
    min_lng, max_lng = sys.float_info.max, -sys.float_info.max

    for point in map_path:
        if point.latitude > max_lat:
            max_lat = point.latitude
        if point.latitude < min_lat:
            min_lat = point.latitude
        if point.longitude > max_lng:
            max_lng = point.longitude
        if point.longitude < min_lng:
            min_lng = point.longitude
    center_lat = (min_lat + max_lat) / 2.0
    center_lng = (min_lng + max_lng) / 2.0
    zoom = int(math.log(1.28 / (max_lat - min_lat)) / math.log(2.0)) + 8

    result = 'var gmap = LoadGoogleMap("{}", {}, {}, {});\n'.format(
        canvas_id, center_lat, center_lng, zoom)
    latlng_list = ['[{},{}]'.format(point.latitude, point.longitude)
                   for point in map_path]
    result += 'var latlng_list = [{}];\n'.format(','.join(latlng_list))
    result += 'DrawPolyline(gmap, latlng_list, "blue", 2);\n'

    start, end = map_path[0], map_path[-1]
    result += 'DrawCircle(gmap, {}, {}, 20, "green");\n'.format(
        start.latitude, start.longitude)
    result += 'DrawCircle(gmap, {}, {}, 20, "red");\n'.format(
        end.latitude, end.longitude)

    return result


def draw_disengagements_on_gmap(task):
    """Draw disengagements on Google map."""
    result = ''
    for dis in task.disengagements:
        info = 'disengage at %.1fs' % (dis.time - task.start_time)
        result += 'DrawInfoWindow(gmap, {}, {}, "{}");\n'.format(
            dis.location.latitude, dis.location.longitude, info)
    return result


def readable_data_size(num):
    """Print data size in readable format."""
    for unit in ['B', 'KB', 'MB', 'GB', 'TB', 'PB', 'EB', 'ZB']:
        if num < 1024.0:
            return '%.2f %s' % (num, unit)
        num /= 1024.0
    return "%.2f %s" % (num, 'YB')


# To be registered into jinja2 templates.
utils = {
    'draw_disengagements_on_gmap': draw_disengagements_on_gmap,
    'draw_path_on_gmap': draw_path_on_gmap,
    'find_bag_by_time': find_bag_by_time,
    'find_bag_offset_by_time': find_bag_offset_by_time,
    'loop_type': loop_type,
    'readable_data_size': readable_data_size,
    'task_id_to_path': task_id_to_path,
    'timestamp_to_time': timestamp_to_time,
}
