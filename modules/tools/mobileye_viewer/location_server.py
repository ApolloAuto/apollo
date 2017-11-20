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
heading = None
projector = pyproj.Proj(proj='utm', zone=10, ellps='WGS84')


def mobileye_callback(p_mobileye_pb):
    global mobileye_pb
    mobileye_pb = p_mobileye_pb


def localization_callback(localization_pb):
    global lat, lon, heading
    x = localization_pb.pose.position.x
    y = localization_pb.pose.position.y
    heading = localization_pb.pose.heading
    zone = 10
    lat, lon = utm_to_latlng(zone, x, y)


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


@app.route('/', methods=["POST", "GET"])
def current_latlon():
    point = {}
    point['lat'] = lat
    point['lon'] = lon
    points = [point]

    utm_vehicle_x, utm_vehicle_y = projector(lon, lat)
    if mobileye_pb is not None:
        rc0 = mobileye_pb.lka_768.position
        rc1 = mobileye_pb.lka_769.heading_angle
        rc2 = mobileye_pb.lka_768.curvature
        rc3 = mobileye_pb.lka_768.curvature_derivative
        right_lane_marker_range = mobileye_pb.lka_769.view_range
        right_lane_marker_coef = [rc0, rc1, rc2, rc3]
        right_lane = []
        for x in range(int(right_lane_marker_range)):
            y = -1 * polyval(x, right_lane_marker_coef)
            newx = x * math.cos(heading) - y * math.sin(heading)
            newy = y * math.cos(heading) + x * math.sin(heading)

            plon, plat = projector(utm_vehicle_x + newx, utm_vehicle_y + newy,
                                   inverse=True)
            right_lane.append({'lat': plat, 'lng': plon})
        # print right_lane
        points.append(right_lane)

        lc0 = mobileye_pb.lka_766.position
        lc1 = mobileye_pb.lka_767.heading_angle
        lc2 = mobileye_pb.lka_766.curvature
        lc3 = mobileye_pb.lka_766.curvature_derivative
        left_lane_marker_range = mobileye_pb.lka_767.view_range
        left_lane_marker_coef = [lc0, lc1, lc2, lc3]
        left_lane = []
        for x in range(int(left_lane_marker_range)):
            y = -1 * polyval(x, left_lane_marker_coef)
            newx = x * math.cos(heading) - y * math.sin(heading)
            newy = y * math.cos(heading) + x * math.sin(heading)
            plon, plat = projector(utm_vehicle_x + newx, utm_vehicle_y + newy,
                                   inverse=True)
            left_lane.append({'lat': plat, 'lng': plon})
        points.append(left_lane)

    return jsonify(points)


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


def utm_to_latlng(zone, easting, northing, northernHemisphere=True):
    """
    this function is cited from stackoverflow
    the approach is stated in
    https://www.ibm.com/developerworks/java/library/j-coordconvert/index.html
    """
    if not northernHemisphere:
        northing = 10000000 - northing

    a = 6378137
    e = 0.081819191
    e1sq = 0.006739497
    k0 = 0.9996

    arc = northing / k0
    mu = arc / (a * (
        1 - math.pow(e, 2) / 4.0 - 3 * math.pow(e, 4)
        / 64.0 - 5 * math.pow(e, 6) / 256.0))

    ei = (1 - math.pow((1 - e * e), (1 / 2.0))) / (
        1 + math.pow((1 - e * e), (1 / 2.0)))

    ca = 3 * ei / 2 - 27 * math.pow(ei, 3) / 32.0

    cb = 21 * math.pow(ei, 2) / 16 - 55 * math.pow(ei, 4) / 32
    cc = 151 * math.pow(ei, 3) / 96
    cd = 1097 * math.pow(ei, 4) / 512
    phi1 = mu + ca * math.sin(2 * mu) + cb * math.sin(4 * mu) + cc * math.sin(
        6 * mu) + cd * math.sin(8 * mu)

    n0 = a / math.pow((1 - math.pow((e * math.sin(phi1)), 2)), (1 / 2.0))

    r0 = a * (1 - e * e) / math.pow((1 - math.pow((e * math.sin(phi1)), 2)),
                                    (3 / 2.0))
    fact1 = n0 * math.tan(phi1) / r0

    _a1 = 500000 - easting
    dd0 = _a1 / (n0 * k0)
    fact2 = dd0 * dd0 / 2

    t0 = math.pow(math.tan(phi1), 2)
    Q0 = e1sq * math.pow(math.cos(phi1), 2)
    fact3 = (5 + 3 * t0 + 10 * Q0 - 4 * Q0 * Q0 - 9 * e1sq) * \
            math.pow(dd0, 4) / 24

    fact4 = (61 + 90 * t0 + 298 * Q0 + 45 * t0 * t0 - 252 * e1sq - 3 * Q0 * Q0) \
            * math.pow(dd0, 6) / 720

    lof1 = _a1 / (n0 * k0)
    lof2 = (1 + 2 * t0 + Q0) * math.pow(dd0, 3) / 6.0
    lof3 = (5 - 2 * Q0 + 28 * t0 - 3 * math.pow(Q0,
                                                2) + 8 * e1sq + 24 * math.pow(
        t0, 2)) * math.pow(dd0, 5) / 120
    _a2 = (lof1 - lof2 + lof3) / math.cos(phi1)
    _a3 = _a2 * 180 / math.pi

    latitude = 180 * (phi1 - fact1 * (fact2 + fact3 + fact4)) / math.pi

    if not northernHemisphere:
        latitude = -latitude

    longitude = ((zone > 0) and (6 * zone - 183.0) or 3.0) - _a3

    return latitude, longitude


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
    app.run(debug=False, port=5001, host='localhost')


if __name__ == "__main__":
    f = open(os.path.dirname(os.path.abspath(__file__)) +
             "/location_server_key", 'r')
    for line in f:
        API_KEY = line.replace('\n', "")
    f.close()

    add_listener()
    thread.start_new_thread(run_flask, ())
    # app.run(debug=False, port=5001, host='localhost')
    rospy.spin()
