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
import pytz
import sys

import gflags

from modules.common.proto.drive_event_pb2 import DriveEvent

gflags.DEFINE_string('timezone', 'America/Los_Angeles', 'Timezone.')


def timestamp_to_time(timestamp):
    """Convert Unix epoch timestamp to readable time."""
    dt = datetime.datetime.fromtimestamp(timestamp, pytz.utc)
    local_tz = pytz.timezone(gflags.FLAGS.timezone)
    return dt.astimezone(local_tz).strftime('%Y-%m-%d %H:%M:%S')


def timestamp_ns_to_time(timestamp):
    """Convert Unix epoch timestamp to readable time."""
    return timestamp_to_time(timestamp / 1e9)


def draw_path_on_gmap(driving_path, canvas_id):
    """Draw driving path on Google map."""
    if not driving_path:
        return ''
    # Get center and zoom.
    min_lat, max_lat = sys.float_info.max, -sys.float_info.max
    min_lng, max_lng = sys.float_info.max, -sys.float_info.max

    for point in driving_path:
        if point.lat > max_lat:
            max_lat = point.lat
        if point.lat < min_lat:
            min_lat = point.lat
        if point.lon > max_lng:
            max_lng = point.lon
        if point.lon < min_lng:
            min_lng = point.lon
    center_lat = (min_lat + max_lat) / 2.0
    center_lng = (min_lng + max_lng) / 2.0
    zoom = int(math.log(1.28 / (max_lat - min_lat + 0.001)) / math.log(2.0)) + 8

    result = 'var gmap = LoadGoogleMap("{}", {}, {}, {});\n'.format(
        canvas_id, center_lat, center_lng, zoom)
    latlng_list = ['[{},{}]'.format(point.lat, point.lon)
                   for point in driving_path]
    result += 'var latlng_list = [{}];\n'.format(','.join(latlng_list))
    result += 'DrawPolyline(gmap, latlng_list, "blue", 2);\n'

    start, end = driving_path[0], driving_path[-1]
    result += 'DrawCircle(gmap, {}, {}, 20, "green");\n'.format(
        start.lat, start.lon)
    result += 'DrawCircle(gmap, {}, {}, 20, "red");\n'.format(end.lat, end.lon)

    return result


def draw_disengagements_on_gmap(record):
    """Draw disengagements on Google map."""
    result = ''
    for dis in record.disengagements:
        info = 'disengage at %.1fs' % (
            dis.time - record.header.begin_time / 1e9)
        result += 'DrawInfoWindow(gmap, {}, {}, "{}");\n'.format(
            dis.location.lat, dis.location.lon, info)
    return result


def readable_data_size(num):
    """Print data size in readable format."""
    for unit in ['B', 'KB', 'MB', 'GB', 'TB', 'PB', 'EB', 'ZB']:
        if num < 1024.0:
            return '%.2f %s' % (num, unit)
        num /= 1024.0
    return "%.2f %s" % (num, 'YB')


def drive_event_type_name(event_type):
    """Convert DriveEvent type to name."""
    return DriveEvent.Type.Name(event_type)


# To be registered into jinja2 templates.
utils = {
    'draw_disengagements_on_gmap': draw_disengagements_on_gmap,
    'draw_path_on_gmap': draw_path_on_gmap,
    'readable_data_size': readable_data_size,
    'timestamp_to_time': timestamp_to_time,
    'timestamp_ns_to_time': timestamp_ns_to_time,
    'drive_event_type_name': drive_event_type_name,
}
