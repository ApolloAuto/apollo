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
"""
Create base map from localization and mobileye lane detection
"""

import argparse
import csv
import math
import numpy as np
import os
import rospy
import sys

import gflags
FLAGS = gflags.FLAGS

from modules.map.proto.map_pb2 import Map
from modules.map.proto.map_lane_pb2 import LaneBoundaryType, Lane
from modules.map.proto.map_road_pb2 import BoundaryEdge

gflags.DEFINE_string('map_dir', 'modules/map/data/demo', 'output map directory')

class DataPoint:
	def __init__(self):
		self.pos_x = 0.0
		self.pos_y = 0.0
		self.pos_z = 0.0
		self.theta = 0.0
		self.dist_left = 0.0
		self.conf_left = 0
		self.dist_right = 0.0
		self.conf_right = 0
		self.width = 0.0
		self.ratio = 0.0

def distance(x1, y1, x2, y2):
	return math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))

def main():
	parser = argparse.ArgumentParser(
		description='Generate Base Map from Recorded Localization and Mobileye Lane Detection')
	parser.add_argument(
		'-i',
		'--input_file',
		help='Recorded localization and mobileye lane detection in CSV format',
		type=str,
		default='lane.csv')
	parser.add_argument(
		'-o',
		'--output_file',
		help='Output file name of generated base map',
		type=str,
		default='base_map.txt')
	args = vars(parser.parse_args())

	csv_file_name = args['input_file']
	map_file_name = args['output_file']

	rows = []
	with open(csv_file_name, 'r') as csvfile:
		reader = csv.reader(csvfile)
		for row in reader:
			rows.append(row)

	data = []
	for row in rows[3201:-3000]:
		entry = DataPoint()
		entry.pos_x = float(row[0])
		entry.pos_y = float(row[1])
		entry.pos_z = float(row[2])
		entry.theta = float(row[3])
		entry.dist_left = abs(float(row[4]))
		entry.conf_left = int(row[5])
		entry.dist_right = abs(float(row[6]))
		entry.conf_right = int(row[7])
		entry.width = 0.0
		entry.ratio = 0.0
		data.append(entry)

	# width = 3.5

	map = Map()
	map.header.version = "1.400000"
	map.header.date = "20170919"
	map.header.district = "101"
	# map.header.projection.proj = "+proj=tmerc +lat_0={37.413082} +lon_0={-122.013332} +k={0.9999999996} +ellps=WGS84 +no_defs"

	total_length = 0.0
	sample_distance = 0.2

	lane = map.lane.add()
	lane.id.id = "84_1_-1"
	central_curve_seg = lane.central_curve.segment.add()
	central_curve_seg.s = 0.0

	left_boundary_curve_seg = lane.left_boundary.curve.segment.add()
	left_boundary_curve_seg.s = 0.0

	right_boundary_curve_seg = lane.right_boundary.curve.segment.add()
	right_boundary_curve_seg.s = 0.0

	last_x = 0.0
	last_y = 0.0

	for (index, entry) in enumerate(data):
		pos_x = entry.pos_x
		pos_y = entry.pos_y
		pos_z = entry.pos_z
		theta = entry.theta
		dist_left = entry.dist_left
		dist_right = entry.dist_right

		# fixed width
		# dist_left = width / 2.0
		# dist_right = width / 2.0

		theta_left = theta + np.pi / 2.0
		pos_l_x = pos_x + dist_left * np.cos(theta_left)
		pos_l_y = pos_y + dist_left * np.sin(theta_left)

		theta_right = theta - np.pi / 2.0
		pos_r_x = pos_x + dist_right * np.cos(theta_right)
		pos_r_y = pos_y + dist_right * np.sin(theta_right)

		pos_c_x = (pos_l_x + pos_r_x) / 2.0
		pos_c_y = (pos_l_y + pos_r_y) / 2.0

		if index == 0:
			central_curve_seg.start_position.x = pos_c_x
			central_curve_seg.start_position.y = pos_c_y

			left_boundary_curve_seg.start_position.x = pos_l_x
			left_boundary_curve_seg.start_position.y = pos_l_y

			right_boundary_curve_seg.start_position.x = pos_r_x
			right_boundary_curve_seg.start_position.y = pos_r_y
		else:
			dist = distance(last_x, last_y, pos_c_x, pos_c_y)
			if dist <= sample_distance:
				continue
			total_length += dist

		point = central_curve_seg.line_segment.point.add()
		point.x = pos_c_x
		point.y = pos_c_y

		point = left_boundary_curve_seg.line_segment.point.add()
		point.x = pos_l_x
		point.y = pos_l_y

		point = right_boundary_curve_seg.line_segment.point.add()	
		point.x = pos_r_x
		point.y = pos_r_y

		sample = lane.left_sample.add()
		sample.s = total_length
		sample.width = dist_left

		sample = lane.right_sample.add()
		sample.s = total_length
		sample.width = dist_right

		last_x = pos_c_x
		last_y = pos_c_y

	central_curve_seg.length = total_length

	left_boundary_curve_seg.heading = float(data[0].theta)
	left_boundary_curve_seg.length = total_length

	right_boundary_curve_seg.heading = float(data[0].theta)
	right_boundary_curve_seg.length = total_length

	boundary_type = lane.left_boundary.boundary_type.add()
	boundary_type.s = 0.0
	boundary_type.types.append(LaneBoundaryType.DOTTED_YELLOW)

	lane.left_boundary.length = total_length

	boundary_type = lane.right_boundary.boundary_type.add()
	boundary_type.s = 0.0
	boundary_type.types.append(LaneBoundaryType.CURB)

	lane.right_boundary.length = total_length

	lane.length = total_length
	lane.speed_limit = 45
	lane.type = Lane.CITY_DRIVING
	lane.turn = Lane.NO_TURN

	road = map.road.add()
	road.id.id = "84"
	section = road.section.add()
	section.id.id = "1"
	section.lane_id.add().id = "84_1_-1"

	left_edge = section.boundary.outer_polygon.edge.add()
	left_edge.type = BoundaryEdge.LEFT_BOUNDARY
	left_edge.curve.CopyFrom(lane.left_boundary.curve)

	right_edge = section.boundary.outer_polygon.edge.add()
	right_edge.type = BoundaryEdge.RIGHT_BOUNDARY
	right_edge.curve.CopyFrom(lane.right_boundary.curve)

	with open(map_file_name, "w") as f:
		f.write(map.__str__())

if __name__ == '__main__':
    main()
