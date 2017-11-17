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
from numpy.polynomial.polynomial import polyval
from modules.localization.proto import localization_pb2
from modules.planning.proto import planning_pb2
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
projector = pyproj.Proj(proj='utm', zone=10, ellps='WGS84')


def add_listener():
    global routing_pub
    rospy.init_node("navigation_map_routing", anonymous=True)
    routing_pub = rospy.Publisher('/apollo/navigation/routing',
                                  String, queue_size=1)


@app.route('/routing', methods=["POST", "GET"])
def routing():
    content = request.json
    start_latlon = str(content["start_lat"]) + "," + str(content["start_lon"])
    end_latlon = str(content["end_lat"]) + "," + str(content["end_lon"])

    url = "https://maps.googleapis.com/maps/api/directions/json?origin=" + \
          start_latlon + "&destination=" + end_latlon + \
          "&key=" + API_KEY
    res = requests.get(url)
    path = []
    if res.status_code != 200:
        return jsonify(path)
    response = json.loads(res.content)

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

    add_listener()
    thread.start_new_thread(run_flask, ())
    # app.run(debug=False, port=5001, host='localhost')
    rospy.spin()
