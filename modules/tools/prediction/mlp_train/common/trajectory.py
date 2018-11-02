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

import abc
import logging
import math
import numpy as np

from configure import parameters
from common.bounding_rectangle import BoundingRectangle
param_fea = parameters['feature']


class TrajectoryToSample(object):

    __metaclass__ = abc.ABCMeta

    def __call__(self, trajectory):
        self.clean(trajectory)
        self.label(trajectory)
        data = self.pack(trajectory)
        return data

    @staticmethod
    def clean(trajectory):
        '''
        Clean up the feature points when lane_id changing abruptly,
        meaning that if the lane_id of current timestamp is different
        from that of the previous one and that of the next one, remove
        this contaminated data.
        '''
        results = []
        traj_len = len(trajectory)
        for i in range(traj_len - 2, 0, -1):
            if not trajectory[i].HasField('lane') or \
               not trajectory[i].lane.HasField('lane_feature'):
                continue

            lane_seq_sz = len(trajectory[i].lane.lane_graph.lane_sequence)
            if lane_seq_sz == 0:
                continue
            elif lane_seq_sz > 10:
                print "Too many lane sequences:", lane_seq_sz

            fea_prev = trajectory[i - 1]
            fea_curr = trajectory[i]
            fea_post = trajectory[i + 1]

            if fea_prev.HasField('lane') and \
               fea_prev.lane.HasField('lane_feature'):
                lane_id_prev = fea_prev.lane.lane_feature.lane_id
            else:
                continue

            if fea_curr.HasField('lane') and \
               fea_curr.lane.HasField('lane_feature'):
                lane_id_curr = fea_curr.lane.lane_feature.lane_id
            else:
                continue

            if fea_post.HasField('lane') and \
               fea_post.lane.HasField('lane_feature'):
                lane_id_post = fea_post.lane.lane_feature.lane_id
            else:
                continue

            if lane_id_curr == lane_id_prev or lane_id_curr == lane_id_post:
                results.append(trajectory[i])

        results.reverse()
        return results

    @staticmethod
    def cmp_lane_seq(real_seq, predict_seq):
        '''
        -1: False Cutin
        0:  False Go
        1:  True Go
        2:  True Cutin
        '''
        if real_seq[0] == predict_seq[0]:
            for i in range(1, len(real_seq)):
                if len(real_seq) > len(predict_seq):
                    return 0
                if real_seq[i] != predict_seq[i]:
                    return 0
            return 1

        else:
            if len(real_seq) == 1:
                return -1

            for i in range(1, len(real_seq)):
                if len(real_seq) - 1 > len(predict_seq):
                    return -1
                if real_seq[i] != predict_seq[i - 1]:
                    return -1
            return 2

    def is_successor_lane(self, feature, lane_id):
        '''
        return True if lane_id is the successor lane of feature
        '''
        if feature.HasField('lane') and \
           feature.lane.HasField('lane_graph') and \
           len(feature.lane.lane_graph.lane_sequence) > 0:
            for lane_seq in feature.lane.lane_graph.lane_sequence:
                seq_lane_ids = []
                for lan_seq in lane_seq.lane_segment:
                    seq_lane_ids.append(lane_seg.lane_id)
                if feature.lane.lane_feature.lane_id in seq_lane_ids and \
                   lane_id in seq_lane_ids:
                    return True
            return False
        else:
            return True

    @classmethod
    def label(cls, trajectory):
        '''
        label feature trajectory according to real future lane sequence in 3s
        '''
        traj_len = len(trajectory)
        for i, fea in enumerate(trajectory):
            if not fea.HasField('lane') or \
               not fea.lane.HasField('lane_feature'):
                print "No lane feature, cancel labeling"
                continue

            future_lane_ids = []
            for j in range(i, traj_len):
                time_span = trajectory[j].timestamp - fea.timestamp
                if time_span > param_fea['prediction_label_timeframe']:
                    break
                if not trajectory[j].HasField('lane') or \
                   not trajectory[j].lane.HasField('lane_feature'):
                    continue

                lane_id_j = trajectory[j].lane.lane_feature.lane_id
                trajectory[i].label_update_time_delta = time_span
                if lane_id_j in future_lane_ids:
                    continue
                else:
                    future_lane_ids.append(lane_id_j)

            if len(future_lane_ids) < 1:
                print "No lane id"
                continue

            seq_size = len(fea.lane.lane_graph.lane_sequence)
            for j in range(seq_size):
                seq = fea.lane.lane_graph.lane_sequence[j]
                if len(seq.lane_segment) == 0:
                    continue
                predict_lane_ids = []
                for k in range(len(seq.lane_segment)):
                    if seq.lane_segment[k].HasField('lane_id'):
                        predict_lane_ids.append(seq.lane_segment[k].lane_id)

                seq.label = cls.cmp_lane_seq(future_lane_ids, predict_lane_ids)
        return trajectory

    @classmethod
    def label_cruise(cls, trajectory):
        '''
        Label feature trajectory according to real future lane sequence
        in 6sec
        '''
        feature_seq_len = len(trajectory)
        for i, fea in enumerate(trajectory):
            # Sanity check.
            if not fea.HasField('lane') or \
               not fea.lane.HasField('lane_feature'):
                print "No lane feature, cancel labeling"
                continue

            # Find the lane_sequence at which the obstacle is located,
            # and put all the lane_segment ids into a set.
            curr_lane_seq = set()
            for lane_sequence in fea.lane.lane_graph.lane_sequence:
                if lane_sequence.vehicle_on_lane:
                    for lane_segment in lane_sequence.lane_segment:
                        curr_lane_seq.add(lane_segment.lane_id)

            if len(curr_lane_seq) == 0:
                print "Obstacle is not on any lane."
                continue

            new_lane_id = None
            has_started_lane_change = False
            has_finished_lane_change = False
            lane_change_start_time = None
            lane_change_finish_time = 10.0

            # Go through all the subsequent features in this sequence
            # of features.
            for j in range(i, feature_seq_len):
                # If timespan exceeds max. maneuver finish time, then break.
                time_span = trajectory[j].timestamp - fea.timestamp
                if time_span > param_fea['maximum_maneuver_finish_time']:
                    break

                # Sanity check.
                if not trajectory[j].HasField('lane') or \
                   not trajectory[j].lane.HasField('lane_feature'):
                    continue
                fea.label_update_time_delta = time_span

                # If step into another lane, label lane change to be started.
                lane_id_j = trajectory[j].lane.lane_feature.lane_id
                if lane_id_j not in curr_lane_seq:
                    if has_started_lane_change == False:
                        has_started_lane_change = True
                        lane_change_start_time = time_span
                        lane_change_finish_time = 10.0
                        new_lane_id = lane_id_j
                else:
                    has_started_lane_change = False
                    new_lane_id = None

                # If roughly get to the center of another lane, label lane change to be finished.
                left_bound = trajectory[j].lane.lane_feature.dist_to_left_boundary
                right_bound = trajectory[j].lane.lane_feature.dist_to_right_boundary
                if left_bound / (left_bound + right_bound) > (0.5 - param_fea['lane_change_finish_condition']) and \
                   left_bound / (left_bound + right_bound) < (0.5 + param_fea['lane_change_finish_condition']):
                    if has_started_lane_change:
                        has_finished_lane_change = True
                        lane_change_finish_time = time_span
                        new_lane_id = lane_id_j
                        # This is to let such data not be eliminated by label_file function
                        fea.label_update_time_delta = param_fea['maximum_maneuver_finish_time']
                        break
                    else:
                        # This means that the obstacle moves back to the center
                        # of the original lane for the first time.
                        if lane_change_finish_time == 10.0:
                            lane_change_finish_time = time_span
            
            '''
            For every lane_sequence in the lane_graph,
            assign a label and a finish_time.
            -1: False Cut-in
            0:  False Follow-lane
            1:  True Follow-lane
            2:  True Cut-in
            3:  True Cut-in but time_to_lane_center unknown (started lane-change but haven't finished)
            4:  True Follow-lane but time_to_lane_center unknown
            '''
            for lane_sequence in fea.lane.lane_graph.lane_sequence:
                if len(lane_sequence.lane_segment) == 0:
                    continue
                # The current lane is obstacle's original lane.
                if lane_sequence.vehicle_on_lane:
                    # Obs is following this lane.
                    if not has_started_lane_change:
                        # Obstacle is following the original lane but is never at lane-center:
                        if lane_change_finish_time == 10.0:
                            lane_sequence.label = 4
                            lane_sequence.time_to_lane_edge = 10.0
                            lane_sequence.time_to_lane_center = 10.0
                        # Obstacle is following the original lane and moved to lane-center
                        else:
                            lane_sequence.label = 1
                            lane_sequence.time_to_lane_edge = 10.0
                            lane_sequence.time_to_lane_center = lane_change_finish_time
                    # Obs has stepped out of this lane within 6sec.
                    else:
                        lane_sequence.label = 0
                        lane_sequence.time_to_lane_edge = lane_change_start_time
                        lane_sequence.time_to_lane_center = 100.0
                # The current lane is NOT obstacle's original lane.
                else:
                    # Obstacle is following the original lane.
                    if not has_started_lane_change:
                        lane_sequence.label = -1
                        lane_sequence.time_to_lane_edge = 100.0
                        lane_sequence.time_to_lane_center = 100.0
                    else:
                        new_lane_id_is_in_this_lane_seq = False
                        for lane_segment in lane_sequence.lane_segment:
                            if lane_segment.lane_id == new_lane_id:
                                new_lane_id_is_in_this_lane_seq = True
                                break
                        # Obstacle has changed to this lane.
                        if new_lane_id_is_in_this_lane_seq:
                            # Obstacle has finished lane changing within 6 sec.
                            if has_finished_lane_change:
                                lane_sequence.label = 2
                                lane_sequence.time_to_lane_edge = lane_change_start_time
                                lane_sequence.time_to_lane_center = lane_change_finish_time
                            # Obstacle started lane changing but haven't finished yet.
                            else:
                                lane_sequence.label = 3
                                lane_sequence.time_to_lane_edge = lane_change_start_time
                                lane_sequence.time_to_lane_center = 10.0

                        # Obstacle has changed to some other lane.
                        else:
                            lane_sequence.label = -1
                            lane_sequence.time_to_lane_edge = 100.0
                            lane_sequence.time_to_lane_center = 100.0

        return trajectory

    @classmethod
    def label_junction(cls, trajectory):
        '''
        label feature trajectory according to real future lane sequence in 7s
        '''
        traj_len = len(trajectory)
        for i, fea in enumerate(trajectory):
            # Sanity check.
            if not fea.HasField('junction_feature') or \
               not len(fea.junction_feature.junction_exit) or \
               not len(fea.junction_feature.junction_mlp_feature):
                # print("No junction_feature, junction_exit, or junction_mlp_feature, not labeling this frame.")
                continue
            curr_pos = np.array([fea.position.x, fea.position.y])
            # Only keep speed > 1
            # TODO(all) consider recovery
            # if fea.speed <= 1:
            #     continue
            heading = math.atan2(fea.raw_velocity.y, fea.raw_velocity.x)
            # Construct dictionary of all exit with dict[exit_lane_id] = np.array(exit_position)
            exit_dict = dict()
            exit_pos_dict = dict()
            for junction_exit in fea.junction_feature.junction_exit:
                if junction_exit.HasField('exit_lane_id'):
                    exit_dict[junction_exit.exit_lane_id] = BoundingRectangle(junction_exit.exit_position.x, junction_exit.exit_position.y, junction_exit.exit_heading, 0.01, junction_exit.exit_width)
                    exit_pos_dict[junction_exit.exit_lane_id] = np.array([junction_exit.exit_position.x, junction_exit.exit_position.y])
            # Searching for up to 100 frames (10 seconds)
            for j in range(i, min(i + 100, traj_len)):
                car_bounding = BoundingRectangle(trajectory[j].position.x, trajectory[j].position.y, math.atan2(trajectory[j].raw_velocity.y, trajectory[j].raw_velocity.x), trajectory[j].length, trajectory[j].width)
                for key, value in exit_dict.items():
                    if car_bounding.overlap(value):
                        exit_pos = exit_pos_dict[key]
                        delta_pos = exit_pos - curr_pos
                        angle = math.atan2(delta_pos[1], delta_pos[0]) - heading
                        d_idx = int((angle / (2.0 * np.pi)) * 12 % 12)
                        label = [0 for idx in range(12)]
                        label[d_idx] = 1
                        fea.junction_feature.junction_mlp_label.extend(label)
                        break
        return trajectory

    @abc.abstractmethod
    def pack(self, trajectory):
        """ abstractmethod"""
        raise NotImplementedError
