#!/usr/bin/env python3

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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

import json
import pyproj
import argparse
from yattag import Doc
import modules.tools.common.proto_utils as proto_utils
from modules.common_msgs.map_msgs import map_pb2


def generate(api_key, left_boundaries, right_boundaries, center_lat,
             center_lon):
    """
    function to generate html code.
    """
    doc, tag, text, line = Doc().ttl()
    doc.asis('<!DOCTYPE html>')
    api_url = 'https://maps.googleapis.com/maps/api/js?key=' + \
              api_key + '&callback=initMap'
    with tag('html'):
        with tag('head'):
            with tag('title'):
                text('Gmap Viewer')
            doc.asis('<meta name="viewport" content="initial-scale=1.0">')
            doc.asis('<meta charset="utf-8">')
            with tag('style'):
                doc.asis('#map { height: 100%; }')
                doc.asis('html, body { height: 100%; margin: 0; padding: 0; }')
        with tag('body'):
            with tag('div', id='map'):
                pass
            with tag('script'):
                doc.asis('\nvar map;\n')
                doc.asis("\nvar colors = ['#e6194b', '#3cb44b', '#ffe119', \
                '#0082c8', '#f58231', '#911eb4', '#46f0f0', '#f032e6', \
                '#d2f53c', '#fabebe', '#008080', '#e6beff', '#aa6e28', \
                '#fffac8', '#800000', '#aaffc3', '#808000', '#ffd8b1', \
                '#000080', '#808080', '#000000']\n")
                doc.asis('function initMap() {\n')
                doc.asis("map = new google.maps.Map(\
                document.getElementById('map'), {\n")
                doc.asis('center: {lat: ' + str(center_lat) +
                         ', lng: ' + str(center_lon) + '},\n')
                doc.asis('zoom: 16\n')
                doc.asis('});\n')
                doc.asis('var left_boundaries = ' +
                         json.dumps(left_boundaries) + ';\n')
                doc.asis("""
                    for (var i = 0; i < left_boundaries.length; i++) {
                        var boundary = new google.maps.Polyline({
                            path: left_boundaries[i],
                            geodesic: true,
                            strokeColor: colors[i % colors.length],
                            strokeOpacity: 1.0,
                            strokeWeight: 3
                        });
                        boundary.setMap(map);
                    }
                """)
                doc.asis('var right_boundaries = ' +
                         json.dumps(right_boundaries) + ';\n')
                doc.asis("""
                    for (var i = 0; i < right_boundaries.length; i++) {
                        var boundary = new google.maps.Polyline({
                            path: right_boundaries[i],
                            geodesic: true,
                            strokeColor: colors[i % colors.length],
                            strokeOpacity: 1.0,
                            strokeWeight: 3
                        });
                        boundary.setMap(map);
                    }
                """)
                doc.asis('}\n')
            doc.asis('<script src="' + api_url + '"></script>')
    html = doc.getvalue()
    return html


def utm2latlon(x, y, pzone=10):
    """
    convert the utm x y to lat and lon
    """
    projector2 = pyproj.Proj(proj='utm', zone=pzone, ellps='WGS84')
    lon, lat = projector2(x, y, inverse=True)
    return lat, lon


def run(gmap_key, map_file, utm_zone):
    """
    read and process map file
    """
    map_pb = map_pb2.Map()
    proto_utils.get_pb_from_file(map_file, map_pb)

    left_boundaries = []
    right_boundaries = []
    center_lat = None
    center_lon = None
    for lane in map_pb.lane:
        for curve in lane.left_boundary.curve.segment:
            if curve.HasField('line_segment'):
                left_boundary = []
                for p in curve.line_segment.point:
                    point = {}
                    lat, lng = utm2latlon(p.x, p.y, utm_zone)
                    if center_lat is None:
                        center_lat = lat
                    if center_lon is None:
                        center_lon = lng
                    point['lat'] = lat
                    point['lng'] = lng
                    left_boundary.append(point)
                left_boundaries.append(left_boundary)
        for curve in lane.right_boundary.curve.segment:
            if curve.HasField('line_segment'):
                right_boundary = []
                for p in curve.line_segment.point:
                    point = {}
                    lat, lng = utm2latlon(p.x, p.y, utm_zone)
                    point['lat'] = lat
                    point['lng'] = lng
                    right_boundary.append(point)
                right_boundaries.append(right_boundary)

    html = generate(gmap_key, left_boundaries, right_boundaries, center_lat,
                    center_lon)

    with open('gmap.html', 'w') as file:
        file.write(html)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Mapshow is a tool to display hdmap info on a map.",
        prog="mapshow.py")

    parser.add_argument(
        "-m", "--map", action="store", type=str, required=True,
        help="Specify the HDMap file in txt or binary format")
    parser.add_argument(
        "-k", "--key", action="store", type=str, required=True,
        help="Specify your google map api key")
    parser.add_argument(
        "-z", "--zone", action="store", type=int, required=True,
        help="Specify utm zone id. e.g, -z 10")

    args = parser.parse_args()

    run(args.key, args.map, args.zone)
