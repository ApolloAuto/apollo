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

from modules.map.proto.map_pb2 import Map
from modules.map.proto.map_lane_pb2 import LaneBoundaryType, Lane
from modules.map.proto.map_road_pb2 import BoundaryEdge

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
	# for row in rows[3201:-3000]:
	for row in rows[1:-1000]:
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

	intervals = []
	interval_begin = -1
	interval_end = -1
	for (index, entry) in enumerate(data):
		if entry.conf_left >= 0 or entry.conf_right >= 0:
			if interval_begin < 0:
				interval_begin = index
			interval_end = index
		else:
			if interval_begin >= 0:
				intervals.append((interval_begin, interval_end))
				interval_begin = -1
				interval_end = -1
			entry.width = entry.dist_left + entry.dist_right

	for interval in intervals:
		for index in range(interval[0], interval[1] + 1):
			width_begin = data[interval[0] - 1].width
			width_end = data[interval[1] + 1].width
			if interval[0] == 0:
				data[index].width = width_end
			elif interval[1] == len(data) - 1:
				data[index].width = width_begin
			else:
				alpha = float(index - interval[0] + 1) / (interval[1] - interval[0] + 2)
				data[index].width = (1.0 - alpha) * width_begin + alpha * width_end

	for (index, entry) in enumerate(data):
		if entry.conf_left >= 0 and entry.conf_right < 0:
			entry.dist_left = entry.width - entry.dist_right
			entry.conf_left = -1
		elif entry.conf_left < 0 and entry.conf_right >= 0:
			entry.dist_right = entry.width - entry.dist_left
			entry.conf_right = -1

	intervals = []
	interval_begin = -1
	interval_end = -1
	for (index, entry) in enumerate(data):
		if entry.conf_left >= 0 and entry.conf_right >= 0:
			if interval_begin < 0:
				interval_begin = index
			interval_end = index
		else:
			if interval_begin >= 0:
				intervals.append((interval_begin, interval_end))
				interval_begin = -1
				interval_end = -1
			entry.ratio = float(entry.dist_left) / entry.width

	for interval in intervals:
		for index in range(interval[0], interval[1] + 1):
			ratio_begin = data[interval[0] - 1].ratio
			ratio_end = data[interval[1] + 1].ratio
			if interval[0] == 0:
				data[index].ratio = ratio_end
			elif interval[1] == len(data) - 1:
				data[index].ratio = ratio_begin
			else:
				alpha = float(index - interval[0] + 1) / (interval[1] - interval[0] + 2)
				data[index].ratio = (1.0 - alpha) * ratio_begin + alpha * ratio_end

	for (index, entry) in enumerate(data):
		if entry.conf_left >= 0 and entry.conf_right >= 0:
			entry.dist_left = entry.width * entry.ratio
			entry.dist_right = entry.width - entry.dist_left
			entry.conf_left = -1
			entry.conf_right = -1

	# width = 3.5

	mp = Map()
	mp.header.version = "1.400000"
	mp.header.date = "20170919"
	mp.header.district = "101"
	# mp.header.projection.proj = "+proj=tmerc +lat_0={37.413082} +lon_0={-122.013332} +k={0.9999999996} +ellps=WGS84 +no_defs"

	road = mp.road.add()
	road.id.id = "road"
	section = road.section.add()
	section.id.id = "section"

	left_edge = section.boundary.outer_polygon.edge.add()
	left_edge.type = BoundaryEdge.LEFT_BOUNDARY

	right_edge = section.boundary.outer_polygon.edge.add()
	right_edge.type = BoundaryEdge.RIGHT_BOUNDARY

	sample_distance = 0.2
	max_lane_length = 100.0

	total_length = 0.0
	current_length = 0.0
	lane_count = 0

	lane = mp.lane.add()
	lane.id.id = "lane_" + str(lane_count)

	lane_central_curve_seg = lane.central_curve.segment.add()

	start_heading = data[0].theta

	lane_left_boundary_curve_seg = lane.left_boundary.curve.segment.add()
	lane_left_boundary_curve_seg.heading = float(start_heading)
	lane_left_boundary_curve_seg.s = 0.0

	left_edge_curve_seg = left_edge.curve.segment.add()
	left_edge_curve_seg.heading = float(start_heading)
	left_edge_curve_seg.s = 0.0

	lane_right_boundary_curve_seg = lane.right_boundary.curve.segment.add()
	lane_right_boundary_curve_seg.heading = float(start_heading)
	lane_right_boundary_curve_seg.s = 0.0

	right_edge_curve_seg = right_edge.curve.segment.add()
	right_edge_curve_seg.heading = float(start_heading)
	right_edge_curve_seg.s = 0.0

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
			lane_central_curve_seg.start_position.x = pos_c_x
			lane_central_curve_seg.start_position.y = pos_c_y

			lane_left_boundary_curve_seg.start_position.x = pos_l_x
			lane_left_boundary_curve_seg.start_position.y = pos_l_y

			left_edge_curve_seg.start_position.x = pos_l_x
			left_edge_curve_seg.start_position.y = pos_l_y

			lane_right_boundary_curve_seg.start_position.x = pos_r_x
			lane_right_boundary_curve_seg.start_position.y = pos_r_y

			right_edge_curve_seg.start_position.x = pos_r_x
			right_edge_curve_seg.start_position.y = pos_r_y
		else:
			dist = distance(last_x, last_y, pos_c_x, pos_c_y)
			if dist <= sample_distance:
				continue
			current_length += dist
			total_length += dist

		point = lane_central_curve_seg.line_segment.point.add()
		point.x = pos_c_x
		point.y = pos_c_y

		point = lane_left_boundary_curve_seg.line_segment.point.add()
		point.x = pos_l_x
		point.y = pos_l_y

		point = left_edge_curve_seg.line_segment.point.add()
		point.x = pos_l_x
		point.y = pos_l_y

		point = lane_right_boundary_curve_seg.line_segment.point.add()	
		point.x = pos_r_x
		point.y = pos_r_y

		point = right_edge_curve_seg.line_segment.point.add()	
		point.x = pos_r_x
		point.y = pos_r_y

		sample = lane.left_sample.add()
		sample.s = current_length
		sample.width = dist_left

		sample = lane.right_sample.add()
		sample.s = current_length
		sample.width = dist_right

		if (current_length > max_lane_length) or (index == len(data) - 1):
			# cut!
			lane_central_curve_seg.length = current_length
			lane_left_boundary_curve_seg.length = current_length
			lane_right_boundary_curve_seg.length = current_length

			boundary_type = lane.left_boundary.boundary_type.add()
			boundary_type.s = 0.0
			boundary_type.types.append(LaneBoundaryType.DOTTED_YELLOW)

			lane.left_boundary.length = current_length

			boundary_type = lane.right_boundary.boundary_type.add()
			boundary_type.s = 0.0
			boundary_type.types.append(LaneBoundaryType.CURB)

			lane.right_boundary.length = current_length

			lane.length = current_length
			lane.speed_limit = 29.06
			lane.type = Lane.CITY_DRIVING
			lane.turn = Lane.NO_TURN

			section.lane_id.add().id = "lane_" + str(lane_count)
			current_length = 0.0

			if index < len(data) - 1:
				lane_count += 1

				new_lane = mp.lane.add()
				lane.successor_id.add().id = "lane_" + str(lane_count)
				new_lane.predecessor_id.add().id = "lane_" + str(lane_count - 1)

				lane = new_lane
				lane.id.id = "lane_" + str(lane_count)

				lane_central_curve_seg = lane.central_curve.segment.add()
				lane_central_curve_seg.s = 0.0
				lane_central_curve_seg.start_position.x = pos_c_x
				lane_central_curve_seg.start_position.y = pos_c_y

				lane_left_boundary_curve_seg = lane.left_boundary.curve.segment.add()
				lane_left_boundary_curve_seg.heading = theta
				lane_left_boundary_curve_seg.s = 0.0
				lane_left_boundary_curve_seg.start_position.x = pos_l_x
				lane_left_boundary_curve_seg.start_position.y = pos_l_y

				lane_right_boundary_curve_seg = lane.right_boundary.curve.segment.add()
				lane_right_boundary_curve_seg.heading = theta
				lane_right_boundary_curve_seg.s = 0.0
				lane_right_boundary_curve_seg.start_position.x = pos_r_x
				lane_right_boundary_curve_seg.start_position.y = pos_r_y

				point = lane_central_curve_seg.line_segment.point.add()
				point.x = pos_c_x
				point.y = pos_c_y

				point = lane_left_boundary_curve_seg.line_segment.point.add()
				point.x = pos_l_x
				point.y = pos_l_y

				point = lane_right_boundary_curve_seg.line_segment.point.add()	
				point.x = pos_r_x
				point.y = pos_r_y

				sample = lane.left_sample.add()
				sample.s = current_length
				sample.width = dist_left

				sample = lane.right_sample.add()
				sample.s = current_length
				sample.width = dist_right

		last_x = pos_c_x
		last_y = pos_c_y

	left_edge_curve_seg.length = total_length
	right_edge_curve_seg.length = total_length

	with open(map_file_name, "w") as f:
		f.write(mp.__str__())

if __name__ == '__main__':
    main()
