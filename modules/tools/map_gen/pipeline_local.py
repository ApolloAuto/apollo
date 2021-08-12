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
from modules.map.proto import map_pb2
from modules.map.proto import map_signal_pb2
from modules.map.proto import map_yield_sign_pb2
from modules.map.proto import map_stop_sign_pb2
from modules.map.proto import map_overlap_pb2
from modules.map.proto import map_crosswalk_pb2
from google.protobuf import text_format
from shapely.geometry import LineString, Point


input_path="/apollo/data/input_data"
output_path="/apollo/data/output_data"
base_map_path = os.path.join(output_path, 'base_map.txt')
files=os.listdir(input_path)
road_junc=dict()
with open(base_map_path, 'r') as fmap:
    map_data = fmap.read()
    map = map_pb2.Map()
    text_format.Parse(map_data, map)
lanes = {}
lanes_map = {}
for lane in map.lane:
    lane_points = []
    lanes_map[lane.id.id] = lane
    for segment in lane.central_curve.segment:
        for point in segment.line_segment.point:
            lane_points.append((point.x, point.y))
    lane_string = LineString(lane_points)
    lanes[lane.id.id] = lane_string

for file in files:
    file_name=file
    file=os.path.join(input_path,file)
    file_name=file_name.replace(".txt","")
    sign_type=file_name.split("_")[0]
    if sign_type=="stop":
        mapp = map_pb2.Map()
        with open(file, 'r') as fsignal:
            signal_data = fsignal.read()
            text_format.Parse(signal_data, mapp)
        fsignal.close()
        signal=mapp.stop_sign
        lines = {}
        for stop_line in signal.stop_line:
            stop_line_points = []
            for segment in stop_line.segment:
                for point in segment.line_segment.point:
                    stop_line_points.append((point.x, point.y))
            stop_line_string = LineString(stop_line_points)
            for lane_id, lane_string in lanes.items():
                p = stop_line_string.intersection(lane_string)
                if type(p) == Point:
                    s = lane_string.project(p)
                    overlap = map.overlap.add()
                    overlap.id.id = str(lane_id) + "_" + str(signal.id.id)
                    obj = overlap.object.add()
                    obj.id.id = signal.id.id
                    obj.stop_sign_overlap_info.CopyFrom(
                        map_overlap_pb2.StopSignOverlapInfo())
                    obj = overlap.object.add()
                    obj.id.id = lane_id
                    obj.lane_overlap_info.start_s = s
                    obj.lane_overlap_info.end_s = s + 0.1
                    obj.lane_overlap_info.is_merge = False

                    signal.overlap_id.add().id = overlap.id.id
                    lanes_map[lane_id].overlap_id.add().id = overlap.id.id
        map.stop_sign.add().CopyFrom(signal)
    elif sign_type=="signal":
        mapp = map_pb2.Map()
        with open(file, 'r') as fsignal:
            signal_data = fsignal.read()
            text_format.Parse(signal_data, mapp)
        fsignal.close()
        signal=mapp.signal[0]
        lines = {}
        for stop_line in signal.stop_line:
            stop_line_points = []
            for segment in stop_line.segment:
                for point in segment.line_segment.point:
                    stop_line_points.append((point.x, point.y))
            stop_line_string = LineString(stop_line_points)
       
        for lane_id, lane_string in lanes.items():
            p = stop_line_string.intersection(lane_string)
            if type(p) == Point:
                s = lane_string.project(p)
                overlap = map.overlap.add()
                overlap.id.id = str(lane_id) + "_" + str(signal.id.id)
                obj = overlap.object.add()
                obj.id.id = signal.id.id
                obj.stop_sign_overlap_info.CopyFrom(
                    map_overlap_pb2.StopSignOverlapInfo())
                obj = overlap.object.add()
                obj.id.id = lane_id
                obj.lane_overlap_info.start_s = s
                obj.lane_overlap_info.end_s = s + 0.1
                obj.lane_overlap_info.is_merge = False
                signal.overlap_id.add().id = overlap.id.id
                lanes_map[lane_id].overlap_id.add().id = overlap.id.id
        map.signal.add().CopyFrom(signal)
    elif sign_type=="yield":
        signal = map_yield_sign_pb2.YieldSign()
        with open(file, 'r') as fsignal:
            signal_data = fsignal.read()
            text_format.Parse(signal_data, signal)
        fsignal.close()
        for stop_line in signal.stop_line:
            stop_line_points = []
            for segment in stop_line.segment:
                for point in segment.line_segment.point:
                    stop_line_points.append((point.x, point.y))
            stop_line_string = LineString(stop_line_points)
            for lane_id, lane_string in lanes.items():
                p = stop_line_string.intersection(lane_string)
                if type(p) == Point:
                    s = lane_string.project(p)
                    overlap = map.overlap.add()
                    overlap.id.id = str(lane_id) + "_" + str(signal.id.id)
                    obj = overlap.object.add()
                    obj.id.id = signal.id.id
                    obj.yield_sign_overlap_info.CopyFrom(
                        map_overlap_pb2.YieldOverlapInfo())
                    obj = overlap.object.add()
                    obj.id.id = lane_id
                    obj.lane_overlap_info.start_s = s
                    obj.lane_overlap_info.end_s = s + 0.1
                    obj.lane_overlap_info.is_merge = False

                    signal.overlap_id.add().id = overlap.id.id
                    lanes_map[lane_id].overlap_id.add().id = overlap.id.id
            yield_=getattr(map,'yield').add()
            yield_.CopyFrom(signal)
    elif sign_type=="crosswalk":
        cw = map.crosswalk.add()
        with open(file, 'r') as fsignal:
            signal_data = fsignal.read()
            text_format.Parse(signal_data, cw)
        fsignal.close()
        for lane_id, lane_string in lanes.items():
            last_point=None
            min_s=999
            max_s=-999
            for ppoint in cw.polygon.point:
                if last_point==None:
                    last_point=cw.polygon.point[-1]
                cw_line=LineString([(last_point.x,last_point.y) ,(ppoint.x,ppoint.y)])
                p=None
                p=cw_line.intersection(lane_string)
                if type(p)==Point:
                    s=lane_string.project(p)
                    print("get p ",p.x," ",p.y,"at jlane ",list(cw_line.coords))
                    if s<min_s:
                        min_s=s
                    if s>max_s:
                        max_s=s
                last_point=ppoint
            if min_s==999:
                continue
            if min_s==max_s:
                if s>lane_string.length/2:
                    max_s=lane_string.length
                else:
                    min_s=0
            overlap = map.overlap.add()
            overlap.id.id = str(lane_id) + "_" + str(cw.id.id)
            obj = overlap.object.add()
            obj.id.id = cw.id.id
            obj.crosswalk_overlap_info.CopyFrom(map_overlap_pb2.
                            CrosswalkOverlapInfo())
            obj = overlap.object.add()
            obj.id.id = lane_id
            obj.lane_overlap_info.start_s = min_s
            obj.lane_overlap_info.end_s = max_s
            obj.lane_overlap_info.is_merge = False
            cw.overlap_id.add().id = overlap.id.id
            lanes_map[lane_id].overlap_id.add().id = overlap.id.id

fmap = open(os.path.join(output_path,"base_map.txt"), 'w')
fmap.write(str(map))
fmap.close()
