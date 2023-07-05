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
This module creates a node and fake perception data based
on json configurations
"""

import argparse
import math
import time

import simplejson

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.common_msgs.basic_msgs.geometry_pb2 import Point3D
from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import PerceptionObstacle
from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import PerceptionObstacles


_s_seq_num = 0
_s_delta_t = 0.1
_s_epsilon = 1e-8


def get_seq_num():
    """
    Return the sequence number
    """
    global _s_seq_num
    _s_seq_num += 1
    return _s_seq_num


def get_velocity(theta, speed):
    """
    Get velocity from theta and speed
    """
    point = Point3D()
    point.x = math.cos(theta) * speed
    point.y = math.sin(theta) * speed
    point.z = 0.0
    return point


def generate_polygon(point, heading, length, width):
    """
    Generate polygon
    """
    points = []
    half_l = length / 2.0
    half_w = width / 2.0
    sin_h = math.sin(heading)
    cos_h = math.cos(heading)
    vectors = [(half_l * cos_h - half_w * sin_h,
                half_l * sin_h + half_w * cos_h),
               (-half_l * cos_h - half_w * sin_h,
                - half_l * sin_h + half_w * cos_h),
               (-half_l * cos_h + half_w * sin_h,
                - half_l * sin_h - half_w * cos_h),
               (half_l * cos_h + half_w * sin_h,
                half_l * sin_h - half_w * cos_h)]
    for x, y in vectors:
        p = Point3D()
        p.x = point.x + x
        p.y = point.y + y
        p.z = point.z
        points.append(p)

    return points


def load_descrptions(files):
    """
    Load description files
    """
    objects = []
    for file in files:
        with open(file, 'r') as fp:
            obstacles = simplejson.loads(fp.read())
            # Multiple obstacle in one file saves as a list[obstacles]
            if isinstance(obstacles, list):
                for obstacle in obstacles:
                    trace = obstacle.get('trace', [])
                    for i in range(1, len(trace)):
                        if same_point(trace[i], trace[i - 1]):
                            print('same trace point found in obstacle: %s' % obstacle["id"])
                            return None
                    objects.append(obstacle)
            else:  # Default case. handles only one obstacle
                obstacle = obstacles
                trace = obstacle.get('trace', [])
                for i in range(1, len(trace)):
                    if same_point(trace[i], trace[i - 1]):
                        print('same trace point found in obstacle: %s' % obstacle["id"])
                        return None
                objects.append(obstacle)

    return objects


def get_point(a, b, ratio):
    """
    Get point from a to b with ratio
    """
    p = Point3D()
    p.x = a[0] + ratio * (b[0] - a[0])
    p.y = a[1] + ratio * (b[1] - a[1])
    p.z = a[2] + ratio * (b[2] - a[2])
    return p


def init_perception(description):
    """
    Create perception from description
    """
    perception = PerceptionObstacle()
    perception.id = description["id"]
    perception.position.x = description["position"][0]
    perception.position.y = description["position"][1]
    perception.position.z = description["position"][2]
    perception.theta = description["theta"]
    perception.velocity.CopyFrom(get_velocity(
        description["theta"], description["speed"]))
    perception.length = description["length"]
    perception.width = description["width"]
    perception.height = description["height"]
    perception.polygon_point.extend(generate_polygon(perception.position,
                                                     perception.theta,
                                                     perception.length,
                                                     perception.width))
    perception.tracking_time = description["tracking_time"]
    perception.type = PerceptionObstacle.Type.Value(description["type"])
    perception.timestamp = cyber_time.Time.now().to_sec()

    return perception


def same_point(a, b):
    """
    Test if a and b are the same point
    """
    return math.fabs(b[0] - a[0]) < _s_epsilon and \
        math.fabs(b[1] - a[1]) < _s_epsilon


def inner_product(a, b):
    """
    Get the a, b inner product
    """
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross_product(a, b):
    """
    Cross product
    """
    return a[0] * b[1] - a[1] * b[0]


def distance(a, b):
    """
    Return distance between a and b
    """
    return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2 + (b[2] - a[2])**2)


def is_within(a, b, c):
    """
    Check if c is in [a, b]
    """
    if b < a:
        b, a = a, b
    return a - _s_epsilon < c < b + _s_epsilon


def on_segment(a, b, c):
    """
    Test if c is in line segment a-b
    """
    ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
    if math.fabs(cross_product(ac, ab)) > _s_epsilon:
        return False
    return is_within(a[0], b[0], c[0]) and is_within(a[1], b[1], c[1]) \
        and is_within(a[2], b[2], c[2])


def linear_project_perception(description, prev_perception):
    """
    Get perception from linear projection of description
    """
    perception = PerceptionObstacle()
    perception = prev_perception
    perception.timestamp = cyber_time.Time.now().to_sec()
    if "trace" not in description:
        return perception
    trace = description["trace"]
    prev_point = (prev_perception.position.x, prev_perception.position.y,
                  prev_perception.position.z)
    delta_s = description["speed"] * _s_delta_t
    for i in range(1, len(trace)):
        if on_segment(trace[i - 1], trace[i], prev_point):
            dist = distance(trace[i - 1], trace[i])
            delta_s += distance(trace[i - 1], prev_point)
            while dist < delta_s:
                delta_s -= dist
                i += 1
                if i < len(trace):
                    dist = distance(trace[i - 1], trace[i])
                else:
                    return init_perception(description)
            ratio = delta_s / dist
            perception.position.CopyFrom(
                get_point(trace[i - 1], trace[i], ratio))
            perception.theta = math.atan2(trace[i][1] - trace[i - 1][1],
                                          trace[i][0] - trace[i - 1][0])
            perception.velocity.CopyFrom(get_velocity(perception.theta, description["speed"]))
            perception.ClearField("polygon_point")
            perception.polygon_point.extend(generate_polygon(perception.position, perception.theta,
                                                             perception.length, perception.width))
            return perception

    return perception


def generate_perception(perception_description, prev_perception):
    """
    Generate perception data
    """
    perceptions = PerceptionObstacles()
    perceptions.header.sequence_num = get_seq_num()
    perceptions.header.module_name = "perception"
    perceptions.header.timestamp_sec = cyber_time.Time.now().to_sec()
    if not perception_description:
        return perceptions
    if prev_perception is None:
        for description in perception_description:
            p = perceptions.perception_obstacle.add()
            p.CopyFrom(init_perception(description))
        return perceptions
    # Linear projection
    description_dict = {desc["id"]: desc for desc in perception_description}
    for obstacle in prev_perception.perception_obstacle:
        description = description_dict[obstacle.id]
        next_obstacle = linear_project_perception(description, obstacle)
        perceptions.perception_obstacle.add().CopyFrom(next_obstacle)
    return perceptions


def perception_publisher(perception_channel, files, period):
    """
    Publisher
    """
    cyber.init()
    node = cyber.Node("perception")
    writer = node.create_writer(perception_channel, PerceptionObstacles)
    perception_description = load_descrptions(files)
    sleep_time = float(period)  # 10Hz
    global _s_delta_t
    _s_delta_t = sleep_time
    perception = None
    while not cyber.is_shutdown():
        perception = generate_perception(perception_description, perception)
        print(str(perception))
        writer.write(perception)
        time.sleep(sleep_time)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="create fake perception obstacles",
                                     prog="replay_perception.py")
    parser.add_argument("files", action="store", type=str, nargs="*",
                        help="obstacle description files")
    parser.add_argument("-c", "--channel", action="store", type=str,
                        default="/apollo/perception/obstacles",
                        help="set the perception channel")
    parser.add_argument("-p", "--period", action="store", type=float, default=0.1,
                        help="set the perception channel publish time duration")
    args = parser.parse_args()

    perception_publisher(args.channel, args.files, args.period)
