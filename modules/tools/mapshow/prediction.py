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
import math
import threading
import numpy as np
from modules.planning.proto import planning_internal_pb2

class Prediction:
    def __init__(self, planning_pb=None):
        """
        Init
        """
        self.prediction_lock = threading.Lock()
        self.prediction_path_data_x = [] # list of list for each prediction path
        self.prediction_path_data_y = []
        self.prediction_polygons = [] # list of polygons
        self.planning_pb = planning_pb

    def update_prediction_pb(self, planning_pb):
        #self.prediction_pb = planning_pb
        # for each trajectory path, update the trajectory
        self.planning_pb = planning_pb

    def compute_prediction_path_data(self):
        #print "     --- computing prediction path BEGIN"
        prediction_path_data_x = []
        prediction_path_data_y = []
        prediction_polygons = []

        for each_prediction_obstacle in self.planning_pb.debug.planning_data.prediction_obstacle:
            each_path_x = []
            each_path_y = []
            each_polygon = []
            if len(each_prediction_obstacle.trajectory) > 1:
                print "WARNING: our clever awesome Obstacle should not have more than one trajectory!!!"
            if len(each_prediction_obstacle.trajectory) != 1:
                print "WARNING: our clever awesome Obstacle should not have more than one trajectory!!!"
            # the only trajectory
            each_prediction_path = each_prediction_obstacle.trajectory[0]
            for each_prediction_path_point in each_prediction_path.trajectory_point:
                each_path_x.append(each_prediction_path_point.path_point.x)
                each_path_y.append(each_prediction_path_point.path_point.y)
            pob = each_prediction_obstacle.perception_obstacle
            # the polygon
            each_polygon = self.get_vehicle_polygon(
                pob.length,
                pob.width,
                [pob.position.x, pob.position.y, pob.position.z],
                pob.theta)
            prediction_path_data_x.append(each_path_x)
            prediction_path_data_y.append(each_path_y)
            prediction_polygons.append(each_polygon)
        self.prediction_lock.acquire()
        self.prediction_path_data_x = prediction_path_data_x
        self.prediction_path_data_y = prediction_path_data_y
        self.prediction_polygons = prediction_polygons
        self.prediction_lock.release()
        #print "     --- computing prediction path END"
    # End of compute_prediction_path_data

    def get_vehicle_polygon(self, length, width, position, heading):
        front_edge_to_center = length / 2.0
        back_edge_to_center = length / 2.0
        left_edge_to_center = width / 2.0
        right_edge_to_center = width / 2.0

        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        #  (p3)  -------- (p0)
        #        | o     |
        #   (p2) -------- (p1)
        p0_x, p0_y = front_edge_to_center, left_edge_to_center
        p1_x, p1_y = front_edge_to_center, -right_edge_to_center
        p2_x, p2_y = -back_edge_to_center, -left_edge_to_center
        p3_x, p3_y = -back_edge_to_center, right_edge_to_center

        p0_x, p0_y = p0_x * cos_h - p0_y * sin_h, p0_x * sin_h + p0_y * cos_h
        p1_x, p1_y = p1_x * cos_h - p1_y * sin_h, p1_x * sin_h + p1_y * cos_h
        p2_x, p2_y = p2_x * cos_h - p2_y * sin_h, p2_x * sin_h + p2_y * cos_h
        p3_x, p3_y = p3_x * cos_h - p3_y * sin_h, p3_x * sin_h + p3_y * cos_h

        [x, y, z] = position
        polygon = []
        polygon.append([p0_x + x, p0_y + y, 0])
        polygon.append([p1_x + x, p1_y + y, 0])
        polygon.append([p2_x + x, p2_y + y, 0])
        polygon.append([p3_x + x, p3_y + y, 0])
        polygon.append([p0_x + x, p0_y + y, 0])
        return polygon