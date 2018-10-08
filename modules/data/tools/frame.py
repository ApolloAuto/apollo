#!/usr/bin/env python

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
"""
Define the data types for Frame and make it searializable with JSON format
"""

import json

class Vector3:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Vector4:
    def __init__(self, x, y, z, i, is_ground):
        self.x = x
        self.y = y
        self.z = z
        self.i = i 
        self.is_ground = is_ground

class Quaternion:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

class GPSPose:
    def __init__(self, lat, lon, bearing):
        self.lat = lat
        self.lon = lon
        self.bearing = bearing

class CameraImage:
    def __init__(self):
        self.timestamp = 0
        self.image_url = ""
        self.position = None #Vector3 
        self.heading = None #Quaternion
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.skew = 0.0
        self.k1 = 0.0
        self.k2 = 0.0
        self.k3 = 0.0
        self.p1 = 0.0
        self.p2 = 0.0

class RadarPoint:
    def __init__(self):
        self.position = None #Vector3 
        self.direction = None #Vector3

class Frame:
    def __init__(self):
        self.device_position = None #Vector3
        self.device_heading = None #Quaternion
        self.device_gps_pose = None #GPSPose
        self.points = [] #list of Vector4
        self.radar_points = [] #list of RadarPoint
        self.images = [] #list of CameraImage
        self.timestamp = 0

    def serialize_instance(self, obj):
        d = { '__classname__' : type(obj).__name__ }
        d.update(vars(obj))
        return d
    
    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__, 
            sort_keys=True, indent=4)
    
    def dump_to_file(self, output_file):
        # TODO: TEST IT
        with open(output_file, 'w') as outfile:
            json.dump(self.to_json(), outfile)

if __name__ == '__main__':
    # TODO: remove the following tests
    p = Frame()
    p.device_position = Vector3(10, 20, 30.00)
    p.device_gps_pose = GPSPose(0.1, 100, 300.05)
    p.images.append(CameraImage())
    p.images[0].heading = Quaternion(0.1, 0.2, 0.3, 0.4)
    #s = json.dumps(p, indent=4)
    s = p.to_json()
    print (s)