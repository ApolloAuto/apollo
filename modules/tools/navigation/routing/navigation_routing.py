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
from std_msgs.msg import String
from flask import jsonify
from flask import Flask
from flask import request
from flask_cors import CORS
from shapely.geometry import LineString, Point
from numpy.polynomial.polynomial import polyval
from modules.localization.proto import localization_pb2
from modules.planning.proto import planning_pb2
from modules.map.relative_map.proto import navigation_pb2
from modules.drivers.proto import mobileye_pb2

# pip install -U flask-cors
# is currently required in docker

app = Flask(__name__)
CORS(app)
lat = 37.415889
lon = -122.014505
API_KEY = ""
routing_pub = None
mobileye_pb = None
planning_pb = None
heading = None
line_ew = None
line_we = None
line_revloop = None

projector = pyproj.Proj(proj='utm', zone=10, ellps='WGS84')


def add_listener():
    global routing_pub
    rospy.init_node("navigation_map_routing", anonymous=True)
    routing_pub = rospy.Publisher('/apollo/navigation',
                                  navigation_pb2.NavigationInfo, queue_size=1)


@app.route('/routing', methods=["POST", "GET"])
def navigation():
    request_json = request.json
    if "start_lat" not in request_json:
        return jsonify([])
    if "start_lon" not in request_json:
        return jsonify([])
    if "end_lat" not in request_json:
        return jsonify([])
    if "end_lon" not in request_json:
        return jsonify([])
    start_latlon = str(request_json["start_lat"]) + "," + \
                   str(request_json["start_lon"])
    end_latlon = str(request_json["end_lat"]) + "," + \
                 str(request_json["end_lon"])
    url = "http://navi-env.axty8vi3ic.us-west-2." \
          "elasticbeanstalk.com/dreamview/navigation?origin=" + \
          start_latlon + "&destination=" + end_latlon + "&heading=0"

    res = requests.get(url)
    if res.status_code != 200:
        return jsonify([])
    # send to ros
    navigation_info = navigation_pb2.NavigationInfo()
    navigation_info.ParseFromString(res.content)
    routing_pub.publish(navigation_info)
    # send to browser
    latlon_path = []
    for path in navigation_info.navigation_path:
        for point in path.path.path_point:
            lons, lats = projector(point.x, point.y, inverse=True)
            latlon_path.append({'lat': lats, 'lng': lons})
    if len(latlon_path) > 0:
        latlon_path[0]['human'] = True
    return jsonify(latlon_path)


def request_routing(request_json):
    if "start_lat" not in request_json:
        return None
    if "start_lon" not in request_json:
        return None
    if "end_lat" not in request_json:
        return None
    if "end_lon" not in request_json:
        return None

    start_latlon = str(request_json["start_lat"]) + "," + \
                   str(request_json["start_lon"])
    end_latlon = str(request_json["end_lat"]) + "," + \
                 str(request_json["end_lon"])

    url = "https://maps.googleapis.com/maps/api/directions/json?origin=" + \
          start_latlon + "&destination=" + end_latlon + "&key=" + API_KEY

    res = requests.get(url)
    if res.status_code != 200:
        return None
    response = json.loads(res.content)
    if len(response['routes']) < 1:
        return None
    steps = response['routes'][0]['legs'][0]['steps']
    return steps


def get_latlon_and_utm_path(steps):
    latlon_path = []

    if len(steps) > 0:
        if steps[0].get('start_location') is not None:
            loc = steps[0]['start_location']
            x, y = projector(loc['lng'], loc['lat'])
            p_start = (x, y)
            loc = steps[-1]['end_location']
            x, y = projector(loc['lng'], loc['lat'])
            p_end = (x, y)
            line = match_drive_data(p_start, p_end)
            if line is not None:
                utm_points = []
                for i in range(int(line.length)):
                    p = line.interpolate(i)
                    lons, lats = projector(p.x, p.y, inverse=True)
                    latlon_path.append({'lat': lats, 'lng': lons})
                    utm_points.append([p.x, p.y])
                steps = []
                steps.append({})
                steps[0]['human'] = True
                steps[0]['polyline'] = {}
                steps[0]['polyline']['points'] = utm_points
                if len(latlon_path) > 0:
                    latlon_path[0]['human'] = True
                return latlon_path, steps

    for step in steps:
        start_loc = step['start_location']
        end_loc = step['end_location']
        latlon_path.append({'lat': start_loc['lat'], 'lng': start_loc['lng']})
        points = decode_polyline(step['polyline']['points'])
        utm_points = []

        for point in points:
            latlon_path.append({'lat': point[0], 'lng': point[1]})
            x, y = projector(point[1], point[0])
            utm_points.append([x, y])

        step['polyline']['points'] = utm_points
        latlon_path.append({'lat': end_loc['lat'], 'lng': end_loc['lng']})
    return latlon_path, steps


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


def load_drive_data():
    global line_ew, line_we, line_revloop
    dir_path = os.path.dirname(os.path.realpath(__file__))
    hanada_ew_file = dir_path + "/" + "hanada_ew"
    hanada_we_file = dir_path + "/" + "hanada_we"
    rev_loop_file = dir_path + "/" + "rev_loop"

    ew = []
    try:
        f = open(hanada_ew_file, 'r')
        for line in f:
            line = line.replace('\n', '')
            vals = line.split(',')
            if len(vals) != 2:
                continue
            x = float(vals[0])
            y = float(vals[1])
            ew.append((x, y))
        line_ew = LineString(ew)
    except:
        line_ew = None

    we = []
    try:
        f = open(hanada_we_file, 'r')
        for line in f:
            line = line.replace('\n', '')
            vals = line.split(',')
            if len(vals) != 2:
                continue
            x = float(vals[0])
            y = float(vals[1])
            we.append((x, y))
        line_we = LineString(we)
    except:
        line_ew = None

    revloop = []
    try:
        f = open(rev_loop_file, 'r')
        for line in f:
            line = line.replace('\n', '')
            vals = line.split(',')
            if len(vals) != 2:
                continue
            x = float(vals[0])
            y = float(vals[1])
            revloop.append((x, y))
        line_revloop = LineString(revloop)
    except:
        line_revloop = None


def match_drive_data(p_start, p_end):
    ps = Point(p_start)
    pe = Point(p_end)
    if line_we is not None:
        ds = line_we.distance(ps)
        de = line_we.distance(pe)
        ss = line_we.project(ps)
        se = line_we.project(pe)
        if ds < 4 and de < 50:
            if ss < se:
                return line_we

    if line_ew is not None:
        ds = line_ew.distance(ps)
        de = line_ew.distance(pe)
        ss = line_ew.project(ps)
        se = line_ew.project(pe)
        if ds < 4 and de < 50:
            if ss < se:
                return line_ew

    if line_revloop is not None:
        ds = line_revloop.distance(ps)
        de = line_revloop.distance(pe)
        ss = line_revloop.project(ps)
        se = line_revloop.project(pe)
        if ds < 6 and de < 50:
            if ss < se:
                return line_revloop

    return None


@app.route('/routing/deprecated', methods=["POST", "GET"])
def routing():
    request_json = request.json
    if "start_lat" not in request_json:
        return jsonify([])
    if "start_lon" not in request_json:
        return jsonify([])
    if "end_lat" not in request_json:
        return jsonify([])
    if "end_lon" not in request_json:
        return jsonify([])

    steps = request_routing(request_json)
    if steps is None:
        routing_pub.publish(json.dumps([]))
        return jsonify([])

    latlon_path, utm_steps = get_latlon_and_utm_path(steps)

    routing_pub.publish(json.dumps(utm_steps))
    return jsonify(latlon_path)


def run_flask():
    app.run(debug=False, port=5002, host='0.0.0.0')


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

    load_drive_data()
    add_listener()
    thread.start_new_thread(run_flask, ())
    # app.run(debug=False, port=5001, host='localhost')
    rospy.spin()
