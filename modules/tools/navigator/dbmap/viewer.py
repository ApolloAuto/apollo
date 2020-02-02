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
import sys

from yattag import Doc
import pyproj

from modules.map.relative_map.proto import navigation_pb2
import common.proto_utils as proto_utils


class DBMapViewer:
    def __init__(self, utm_zone):
        """
        init function
        """
        self.utm_zone = utm_zone
        self.projector = pyproj.Proj(
            proj='utm', zone=self.utm_zone, ellps='WGS84')

        self.navigation_lines = []
        self.center_lat = None
        self.center_lon = None
        self.html = ""

    def utm2latlon(self, x, y):
        """
        convert the utm x y to lat and lon
        """
        lon, lat = self.projector(x, y, inverse=True)
        return lat, lon

    def add(self, dbmap):
        for navigation_path in dbmap.navigation_path:
            navigation_line = []
            for p in navigation_path.path.path_point:
                point = {}
                lat, lng = self.utm2latlon(p.x, p.y)
                if self.center_lat is None:
                    self.center_lat = lat
                if self.center_lon is None:
                    self.center_lon = lng
                point['lat'] = lat
                point['lng'] = lng
                navigation_line.append(point)
            self.navigation_lines.append(navigation_line)

    def generate(self):
        """
        function to generate html code.
        """
        doc, tag, text, line = Doc().ttl()
        doc.asis('<!DOCTYPE html>')
        api_url = 'http://maps.google.com/maps/api/js?sensor=' \
                  'false&callback=initMap'
        with tag('html'):
            with tag('head'):
                with tag('title'):
                    text('Gmap Viewer')
                doc.asis('<meta name="viewport" content="initial-scale=1.0">')
                doc.asis('<meta charset="utf-8">')
                with tag('style'):
                    doc.asis('#map { height: 100%; }')
                    doc.asis(
                        'html, body { height: 100%; margin: 0; padding: 0; }')
            with tag('body'):
                with tag('div', id='map'):
                    pass
                with tag('script'):
                    doc.asis('\nvar map;\n')
                    doc.asis("\nvar colors = ['#e6194b', '#3cb44b', '#ffe119', "
                             "'#0082c8', '#f58231', '#911eb4', '#46f0f0', "
                             "'#f032e6', '#d2f53c', '#fabebe', '#008080', "
                             "'#e6beff', '#aa6e28', '#fffac8', '#800000', "
                             "'#aaffc3', '#808000', '#ffd8b1', '#000080', "
                             "'#808080', '#000000']\n")
                    doc.asis('function initMap() {\n')
                    doc.asis("map = new google.maps.Map("
                             "document.getElementById('map'), {\n")
                    doc.asis('center: {lat: ' + str(self.center_lat) +
                             ', lng: ' + str(self.center_lon) + '},\n')
                    doc.asis('zoom: 16\n')
                    doc.asis('});\n')
                    doc.asis('var navi_lines = ' +
                             json.dumps(self.navigation_lines) + ';\n')
                    doc.asis("""
                        for (var i = 0; i < navi_lines.length; i++) {
                            var boundary = new google.maps.Polyline({
                                path: navi_lines[i],
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
        self.html = doc.getvalue()
        return self.html


if __name__ == "__main__":
    import google.protobuf.text_format as text_format

    if len(sys.argv) < 2:
        print("usage: python map_viewer.py dbmap_file [utm_zone=10]")
        sys.exit(0)

    map_file = sys.argv[1]
    utm_zone = 10
    if len(sys.argv) >= 3:
        utm_zone = int(sys.argv[2])

    dbmap = navigation_pb2.NavigationInfo()
    proto_utils.get_pb_from_file(map_file, dbmap)
    with open(map_file, 'r') as file_in:
        text_format.Merge(file_in.read(), dbmap)
    viewer = DBMapViewer(utm_zone)
    viewer.add(dbmap)
    html = viewer.generate()
    with open('dbmap.html', 'w') as f:
        f.write(html)
