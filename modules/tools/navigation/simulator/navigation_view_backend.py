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

import os
import rospy
import math
import thread
import requests
import json
import pyproj
import urllib3.contrib.pyopenssl
import certifi
import urllib3
from std_msgs.msg import String
from flask import jsonify
from flask import Flask
from flask import request
from flask_cors import CORS
from numpy.polynomial.polynomial import polyval
from modules.localization.proto import localization_pb2
from modules.planning.proto import planning_pb2
from modules.drivers.proto import mobileye_pb2
from modules.perception.proto import perception_obstacle_pb2

# pip install -U flask-cors
# is currently required in docker

app = Flask(__name__)
CORS(app)
lat = 37.415889
lon = -122.014505
API_KEY = ""
routing_pub = None
mobileye_pb = None
perception_pb = None
planning_pb = None
heading = None
projector = pyproj.Proj(proj='utm', zone=10, ellps='WGS84')
urllib3.contrib.pyopenssl.inject_into_urllib3()
http = urllib3.PoolManager(cert_reqs='CERT_REQUIRED', ca_certs=certifi.where())


def mobileye_callback(p_mobileye_pb):
    global mobileye_pb
    mobileye_pb = p_mobileye_pb


def planning_callback(p_planning_pb):
    global planning_pb
    planning_pb = p_planning_pb


def perception_callback(p_perception_pb):
    global perception_pb
    perception_pb = p_perception_pb


def localization_callback(localization_pb):
    global lat, lon, heading
    x = localization_pb.pose.position.x
    y = localization_pb.pose.position.y
    heading = localization_pb.pose.heading
    zone = 10
    lon, lat = projector(x, y, inverse=True)


def add_listener():
    global routing_pub
    rospy.init_node("map_server", anonymous=True)
    rospy.Subscriber('/apollo/localization/pose',
                     localization_pb2.LocalizationEstimate,
                     localization_callback)
    routing_pub = rospy.Publisher('/apollo/navigation/routing',
                                  String, queue_size=1)
    rospy.Subscriber('/apollo/sensor/mobileye',
                     mobileye_pb2.Mobileye,
                     mobileye_callback)
    rospy.Subscriber('/apollo/planning',
                     planning_pb2.ADCTrajectory,
                     planning_callback)
    rospy.Subscriber('/apollo/perception/obstacles',
                     perception_obstacle_pb2.PerceptionObstacles,
                     perception_callback)


@app.route('/', methods=["POST", "GET"])
def current_latlon():
    point = {}
    point['lat'] = lat
    point['lon'] = lon
    points = [point]

    utm_vehicle_x, utm_vehicle_y = projector(lon, lat)

    right_lane = []
    left_lane = []
    if perception_pb is not None:
        right_lane, left_lane = get_lane_markers_from_perception(
            utm_vehicle_x, utm_vehicle_y)
    else:
        if mobileye_pb is not None:
            right_lane, left_lane = get_lane_markers_from_mobileye(
                utm_vehicle_x, utm_vehicle_y)

    points.append(right_lane)
    points.append(left_lane)

    planned_path = []
    if planning_pb is not None:
        for traj_point in planning_pb.trajectory_point:
            x = traj_point.path_point.x
            y = traj_point.path_point.y
            newx = x * math.cos(heading) - y * math.sin(heading)
            newy = y * math.cos(heading) + x * math.sin(heading)
            plon, plat = projector(utm_vehicle_x + newx, utm_vehicle_y + newy,
                                   inverse=True)
            planned_path.append({'lat': plat, 'lng': plon})
    points.append(planned_path)

    return jsonify(points)


def get_lane_markers_from_perception(utm_vehicle_x, utm_vehicle_y):
    right_lane = []
    left_lane = []

    rc0 = perception_pb.lane_marker.right_lane_marker.c0_position
    rc1 = perception_pb.lane_marker.right_lane_marker.c1_heading_angle
    rc2 = perception_pb.lane_marker.right_lane_marker.c2_curvature
    rc3 = perception_pb.lane_marker.right_lane_marker.c3_curvature_derivative
    right_lane_marker_range = perception_pb.lane_marker.right_lane_marker.view_range
    right_lane_marker_coef = [rc0, rc1, rc2, rc3]

    for x in range(int(right_lane_marker_range)):
        y = -1 * polyval(x, right_lane_marker_coef)
        newx = x * math.cos(heading) - y * math.sin(heading)
        newy = y * math.cos(heading) + x * math.sin(heading)
        plon, plat = projector(utm_vehicle_x + newx, utm_vehicle_y + newy,
                               inverse=True)
        right_lane.append({'lat': plat, 'lng': plon})
    # print right_lane

    lc0 = perception_pb.lane_marker.left_lane_marker.c0_position
    lc1 = perception_pb.lane_marker.left_lane_marker.c1_heading_angle
    lc2 = perception_pb.lane_marker.left_lane_marker.c2_curvature
    lc3 = perception_pb.lane_marker.left_lane_marker.c3_curvature_derivative
    left_lane_marker_range = perception_pb.lane_marker.left_lane_marker.view_range
    left_lane_marker_coef = [lc0, lc1, lc2, lc3]

    for x in range(int(left_lane_marker_range)):
        y = -1 * polyval(x, left_lane_marker_coef)
        newx = x * math.cos(heading) - y * math.sin(heading)
        newy = y * math.cos(heading) + x * math.sin(heading)
        plon, plat = projector(utm_vehicle_x + newx, utm_vehicle_y + newy,
                               inverse=True)
        left_lane.append({'lat': plat, 'lng': plon})
    return right_lane, left_lane


def get_lane_markers_from_mobileye(utm_vehicle_x, utm_vehicle_y):
    right_lane = []
    left_lane = []

    rc0 = mobileye_pb.lka_768.position
    rc1 = mobileye_pb.lka_769.heading_angle
    rc2 = mobileye_pb.lka_768.curvature
    rc3 = mobileye_pb.lka_768.curvature_derivative
    right_lane_marker_range = mobileye_pb.lka_769.view_range
    right_lane_marker_coef = [rc0, rc1, rc2, rc3]

    for x in range(int(right_lane_marker_range)):
        y = -1 * polyval(x, right_lane_marker_coef)
        newx = x * math.cos(heading) - y * math.sin(heading)
        newy = y * math.cos(heading) + x * math.sin(heading)
        plon, plat = projector(utm_vehicle_x + newx, utm_vehicle_y + newy,
                               inverse=True)
        right_lane.append({'lat': plat, 'lng': plon})
    # print right_lane

    lc0 = mobileye_pb.lka_766.position
    lc1 = mobileye_pb.lka_767.heading_angle
    lc2 = mobileye_pb.lka_766.curvature
    lc3 = mobileye_pb.lka_766.curvature_derivative
    left_lane_marker_range = mobileye_pb.lka_767.view_range
    left_lane_marker_coef = [lc0, lc1, lc2, lc3]

    for x in range(int(left_lane_marker_range)):
        y = -1 * polyval(x, left_lane_marker_coef)
        newx = x * math.cos(heading) - y * math.sin(heading)
        newy = y * math.cos(heading) + x * math.sin(heading)
        plon, plat = projector(utm_vehicle_x + newx, utm_vehicle_y + newy,
                               inverse=True)
        left_lane.append({'lat': plat, 'lng': plon})
    return right_lane, left_lane


@app.route('/routing', methods=["POST", "GET"])
def routing():
    content = request.json
    start_latlon = str(content["start_lat"]) + "," + str(content["start_lon"])
    end_latlon = str(content["end_lat"]) + "," + str(content["end_lon"])

    url = "https://maps.googleapis.com/maps/api/directions/json?origin=" + \
          start_latlon + "&destination=" + end_latlon + \
          "&key=" + API_KEY
    res = http.request('GET', url)
    path = []
    if res.status != 200:
        return jsonify(path)
    response = json.loads(res.data)

    if len(response['routes']) < 1:
        return jsonify(path)
    steps = response['routes'][0]['legs'][0]['steps']

    for step in steps:
        start_loc = step['start_location']
        end_loc = step['end_location']
        path.append({'lat': start_loc['lat'], 'lng': start_loc['lng']})
        points = decode_polyline(step['polyline']['points'])
        utm_points = []

        for point in points:
            path.append({'lat': point[0], 'lng': point[1]})
            x, y = projector(point[1], point[0])
            utm_points.append([x, y])

        step['polyline']['points'] = utm_points
        path.append({'lat': end_loc['lat'], 'lng': end_loc['lng']})

    routing_pub.publish(json.dumps(steps))
    return jsonify(path)


def decode_polyline(polyline_str):
    index, lat, lng = 0, 0, 0
    coordinates = []
    changes = {'latitude': 0, 'longitude': 0}
    while index < len(polyline_str):
        for unit in ['latitude', 'longitude']:
            shift, result = 0, 0

            while True:
                byte = ord(polyline_str[index]) - 63
                index += 1
                result |= (byte & 0x1f) << shift
                shift += 5
                if not byte >= 0x20:
                    break

            if (result & 1):
                changes[unit] = ~(result >> 1)
            else:
                changes[unit] = (result >> 1)

        lat += changes['latitude']
        lng += changes['longitude']

        coordinates.append((lat / 100000.0, lng / 100000.0))

    return coordinates


def run_flask():
    app.run(debug=False, port=5001, host='0.0.0.0')


if __name__ == "__main__":
    key_file_name = os.path.dirname(os.path.abspath(__file__)) + \
        "/map_api_key/api_key"
    try:
        f = open(key_file_name, 'r')
        with f:
            for line in f:
                API_KEY = line.replace('\n', "")
                break
    except IOError:
        print "Could not read file:", key_file_name

    add_listener()
    thread.start_new_thread(run_flask, ())
    # app.run(debug=False, port=5001, host='localhost')
    rospy.spin()
