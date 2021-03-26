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
import scipy.interpolate
from scipy.interpolate import interp1d
import numpy as np
import simplejson
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from shapely.geometry import LineString, Point
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacles
from modules.prediction.proto.prediction_obstacle_pb2 import PredictionObstacles
from modules.prediction.proto.prediction_obstacle_pb2 import PredictionObstacle
import modules.common.proto.pnc_point_pb2 as pnc_point
from modules.common.proto.geometry_pb2 import Point3D

_s_seq_num = 0
_s_delta_t = 0.1
_s_epsilon = 1e-8
is_static = False
pub_prediction = False


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


def generate_fix_step_pathpoint(x0, y0):
    points = []
    for i in range(0, len(x0)):
        points.append((x0[i], y0[i]))
    path = LineString(points)
    length = path.length
    x0 = []
    y0 = []
    for i in np.arange(0, length, 0.5):
        p = path.interpolate(i)
        x0.append(p.x)
        y0.append(p.y)
    return x0, y0


def expand(trace):
    x0 = [tra[0] for tra in trace]
    y0 = [tra[1] for tra in trace]
    xx = np.array(x0)
    yy = np.array(y0)
    res = np.arange(xx.min(), xx.max(), 0.01)
    f = interp1d(xx, yy, kind='cubic')
    x0 = res
    y0 = f(x0)
    x0, y0 = generate_fix_step_pathpoint(x0, y0)
    trace = []
    for i in range(0, len(x0)):
        trace.append([x0[i], y0[i]])
    return trace


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
                            print('same trace point found in obstacle: %s' %
                                  obstacle["id"])
                            return None
                    obstacle['trace'] = expand(trace)
                    objects.append(obstacle)
            else:  # Default case. handles only one obstacle
                obstacle = obstacles
                trace = obstacle.get('trace', [])
                for i in range(1, len(trace)):
                    if same_point(trace[i], trace[i - 1]):
                        print('same trace point found in obstacle: %s' %
                              obstacle["id"])
                        return None
                obstacle['trace'] = expand(trace)
                objects.append(obstacle)

    return objects


def get_point(a, b, ratio):
    """
    Get point from a to b with ratio
    """
    p = Point3D()
    p.x = a[0] + ratio * (b[0] - a[0])
    p.y = a[1] + ratio * (b[1] - a[1])
    p.z = 0
    return p


def init_perception(description):
    """
    Create perception from description
    """
    perception = PerceptionObstacle()
    perception.id = description["id"]
    tmp = description["trace"]
    perception.position.x = tmp[0][0]
    perception.position.y = tmp[0][1]
    perception.position.z = 0
    perception.theta = math.atan2(tmp[1][1] - tmp[0][1], tmp[1][0] - tmp[0][0])
   # print("id ",perception.id, "theta ",perception.theta,tmp[1][1]-tmp[0][1],tmp[1][0]-tmp[0][0])
   # print(tmp)
    # perception.theta = description["theta"]
    perception.velocity.CopyFrom(get_velocity(
        perception.theta, description["speed"]))
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
    return a[0] * b[0] + a[1] * b[1]


def cross_product(a, b):
    """
    Cross product
    """
    return a[0] * b[1] - a[1] * b[0]


def distance(a, b):
    """
    Return distance between a and b
    """
    return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)


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
    ab = (b[0] - a[0], b[1] - a[1])
    ac = (c[0] - a[0], c[1] - a[1])
    if math.fabs(cross_product(ac, ab)) > _s_epsilon:
        return False
    return is_within(a[0], b[0], c[0]) and is_within(a[1], b[1], c[1])


def generate_prediction_traj(perception, trace, start):
    prediction = PredictionObstacle()
    prediction.timestamp = cyber_time.Time.now().to_sec()

    prediction.is_static = 0
    traj = prediction.trajectory.add()
    traj.probability = 1
    traj_points = traj.trajectory_point
    dist = 0
    end_time = 0
    for t in range(start, len(trace) - 1):
        pt = traj_points.add()
        pt.path_point.x = trace[t][0]
        pt.path_point.y = trace[t][1]
        dist = dist + distance(trace[t - 1], trace[t])
        pt.path_point.z = 0
        pt.path_point.theta = math.atan2(trace[t + 1][1] - trace[t][1],
                                         trace[t + 1][0] - trace[t][0])
        pt.v = (perception.velocity.x**2
                + perception.velocity.x**2)**0.5
        pt.a = 0
        pt.relative_time = dist / pt.v
        end_time = pt.relative_time
    prediction.predicted_period = end_time
    prediction.perception_obstacle.CopyFrom(perception)
    return prediction


def linear_project_perception(description, prev_perception):
    """
    Get perception and prediction from linear projection of description
    """
    perception = PerceptionObstacle()
    perception = prev_perception
    perception.timestamp = cyber_time.Time.now().to_sec()
    if "trace" not in description:
        return perception
    trace = description["trace"]
    prediction = PredictionObstacle()
    prediction.timestamp = cyber_time.Time.now().to_sec()
    prediction = generate_prediction_traj(perception, trace, 1)
    prev_point = (prev_perception.position.x, prev_perception.position.y,
                  prev_perception.position.z)
    if is_static:
        delta_s = description["speed"] * 0
    else:
        delta_s = description["speed"] / 3.6 * _s_delta_t
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
                    perception = init_perception(description)
                    prediction = generate_prediction_traj(perception, trace, 1)
                    return perception, prediction
            ratio = delta_s / dist
            perception.position.CopyFrom(
                get_point(trace[i - 1], trace[i], ratio))
            perception.theta = math.atan2(trace[i][1] - trace[i - 1][1],
                                          trace[i][0] - trace[i - 1][0])
            perception.velocity.x = description["speed"] * \
                math.cos(perception.theta)
            perception.velocity.y = description["speed"] * \
                math.sin(perception.theta)
            perception.ClearField("polygon_point")
            perception.polygon_point.extend(generate_polygon(perception.position, perception.theta,
                                                             perception.length, perception.width))
            prediction = generate_prediction_traj(perception, trace, i)
            return perception, prediction

    return perception.prediciton


def generate_perception(perception_description, prev_perception):
    """
    Generate perception and prediction data
    """
    perceptions = PerceptionObstacles()
    perceptions.header.sequence_num = get_seq_num()
    perceptions.header.module_name = "perception"
    perceptions.header.timestamp_sec = cyber_time.Time.now().to_sec()
    predictions = PredictionObstacles()
    predictions.header.sequence_num = perceptions.header.sequence_num
    predictions.header.module_name = "prediction"
    predictions.header.timestamp_sec = cyber_time.Time.now().to_sec()
    predictions.header.lidar_timestamp = cyber_time.Time.now().to_nsec()
    predictions.perception_error_code = 0
    if not perception_description:
        return perceptions, predictions
    if prev_perception is None:
        for description in perception_description:
            p = perceptions.perception_obstacle.add()
            p.CopyFrom(init_perception(description))
        return perceptions, predictions
    # Linear projection
    description_dict = {desc["id"]: desc for desc in perception_description}
    for obstacle in prev_perception.perception_obstacle:
        description = description_dict[obstacle.id]
        next_obstacle, next_prediction = linear_project_perception(
            description, obstacle)
        perceptions.perception_obstacle.add().CopyFrom(next_obstacle)
        predictions.prediction_obstacle.add().CopyFrom(next_prediction)
    return perceptions, predictions


def perception_publisher(perception_channel, files, period):
    """
    Publisher
    """
    cyber.init()
    node = cyber.Node("perception")
    writer_prediction = node.create_writer(
        "/apollo/prediction", PredictionObstacles)
    writer_perception = node.create_writer(
        perception_channel, PerceptionObstacles)
    perception_description = load_descrptions(files)
    sleep_time = float(period)  # 10Hz
    global _s_delta_t, is_static, pub_prediction
    _s_delta_t = period
    perception = None
    prediction = None
    while not cyber.is_shutdown():
        perception, prediction = generate_perception(
            perception_description, perception)
        print(str(perception))
        writer_perception.write(perception)
        print(str(prediction))
        if pub_prediction:
            writer_prediction.write(prediction)
        time.sleep(sleep_time)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="create fake perception(prediction ) obstacles",
                                     prog="replay_perception.py")
    parser.add_argument("files", action="store", type=str, nargs="*",
                        help="obstacle description files")
    parser.add_argument("-c", "--channel", action="store", type=str,
                        default="/apollo/perception/obstacles",
                        help="set the perception channel")
    parser.add_argument("-p", "--period", action="store", type=float, default=0.1,
                        help="set the perception channel publish time duration")
    parser.add_argument("-s", "--static", action="store_true", default=False,
                        help="obstacles will not move if set true")
    parser.add_argument("--prediction", action="store_true", default=False,
                        help="publish prediction at the same time if set true")
    args = parser.parse_args()
    is_static, pub_prediction
    is_static = args.static
    pub_prediction = args.prediction
    perception_publisher(args.channel, args.files, args.period)
