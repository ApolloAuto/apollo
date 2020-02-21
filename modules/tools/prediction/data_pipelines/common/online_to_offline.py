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

import abc
import logging
import math

from google.protobuf.internal import decoder
from google.protobuf.internal import encoder
import numpy as np

from modules.prediction.proto import offline_features_pb2
from .bounding_rectangle import BoundingRectangle
from .configure import parameters


param_fea = parameters['feature']


class LabelGenerator(object):

    def __init__(self):
        self.filepath = None
        '''
        feature_dict contains the organized Feature in the following way:
            obstacle_ID --> [Feature1, Feature2, Feature3, ...] (sequentially sorted)
        '''
        self.feature_dict = dict()

        '''
        observation_dict contains the important observations of the subsequent
        Features for each obstacle at every timestamp:
            obstacle_ID@timestamp --> dictionary of observations
        where dictionary of observations contains:
            'obs_traj': the trajectory points (x, y, vel_heading) up to
                        max_observation_time this trajectory poitns must
                        be consecutive (0.1sec sampling period)
            'obs_traj_len': length of trajectory points
            'obs_actual_lane_ids': the actual sequence of lane_segment ids
                                   the obstacle steps on
            'has_started_lane_change': whether the obstacle has started lane
                                       changing within max_observation_time
            'has_finished_lane_change': whether it has finished lane changing
            'lane_change_start_time':
            'lane_change_finish_time':
            'is_jittering':
            'new_lane_id':
            'total_observed_time_span':
        This observation_dict, once constructed, can be reused by various labeling
        functions.
        '''
        self.observation_dict = dict()
        self.future_status_dict = dict()
        self.cruise_label_dict = dict()
        self.junction_label_dict = dict()

    def LoadFeaturePBAndSaveLabelFiles(self, input_filepath):
        '''
        This function will be used to replace all the functionalities in
        generate_cruise_label.py
        '''
        self.filepath = input_filepath
        feature_sequences = self.LoadPBFeatures(input_filepath)
        self.OrganizeFeatures(feature_sequences)
        del feature_sequences  # Try to free up some memory
        self.ObserveAllFeatureSequences()

    '''
    @brief: parse the pb file of Feature of all obstacles at all times.
    @input filepath: the path of the pb file that contains all the features of
                     every obstacle at every timestamp.
    @output: python readable format of the same content.
    '''

    def LoadPBFeatures(self, filepath):
        self.filepath = filepath
        offline_features = offline_features_pb2.Features()
        with open(filepath, 'rb') as file_in:
            offline_features.ParseFromString(file_in.read())
        return offline_features.feature

    '''
    @brief: save the feature_sequences to an output protobuf file.
    @input filepath: the path of the output pb file.
    @input feature_sequences: the content to be saved.
    '''
    @staticmethod
    def SaveOutputPB(filepath, pb_message):
        with open(filepath, 'wb') as file:
            serializedMessage = pb_message.SerializeToString()
            file.write(serializedMessage)

    '''
    @brief: organize the features by obstacle IDs first, then sort each
            obstacle's feature according to time sequence.
    @input features: the unorganized features
    @output: organized (meaning: grouped by obstacle ID and sorted by time)
             features.
    '''

    def OrganizeFeatures(self, features):
        # Organize Feature by obstacle_ID (put those belonging to the same obstacle together)
        for feature in features:
            if feature.id in self.feature_dict.keys():
                self.feature_dict[feature.id].append(feature)
            else:
                self.feature_dict[feature.id] = [feature]

        # For the same obstacle, sort the Feature sequentially.
        for obs_id in self.feature_dict.keys():
            if len(self.feature_dict[obs_id]) < 2:
                del self.feature_dict[obs_id]
                continue
            self.feature_dict[obs_id].sort(key=lambda x: x.timestamp)

    '''
    @brief: observe all feature sequences and build observation_dict.
    @output: the complete observation_dict.
    '''

    def ObserveAllFeatureSequences(self):
        for obs_id, feature_sequence in self.feature_dict.items():
            for idx, feature in enumerate(feature_sequence):
                if not feature.HasField('lane') or \
                   not feature.lane.HasField('lane_feature'):
                    # print('No lane feature, cancel labeling')
                    continue
                self.ObserveFeatureSequence(feature_sequence, idx)
        np.save(self.filepath + '.npy', self.observation_dict)

    '''
    @brief: Observe the sequence of Features following the Feature at
            idx_curr and save some important observations in the class
            so that it can be used by various label functions.
    @input feature_sequence: A sorted sequence of Feature corresponding to
                             one obstacle.
    @input idx_curr: The index of the current Feature to be labelled.
                     We will look at the subsequent Features following this
                     one to complete labeling.
    @output: All saved as class variables in observation_dict,
             including: its trajectory info and its lane changing info.
    '''

    def ObserveFeatureSequence(self, feature_sequence, idx_curr):
        # Initialization.
        feature_curr = feature_sequence[idx_curr]
        dict_key = "{}@{:.3f}".format(feature_curr.id, feature_curr.timestamp)
        if dict_key in self.observation_dict.keys():
            return

        # Record all the lane segments belonging to the lane sequence that the
        # obstacle is currently on.
        curr_lane_segments = set()
        for lane_sequence in feature_curr.lane.lane_graph.lane_sequence:
            if lane_sequence.vehicle_on_lane:
                for lane_segment in lane_sequence.lane_segment:
                    curr_lane_segments.add(lane_segment.lane_id)
        if len(curr_lane_segments) == 0:
            # print("Obstacle is not on any lane.")
            return

        # Declare needed varables.
        new_lane_id = None
        has_started_lane_change = False
        has_finished_lane_change = False
        lane_change_start_time = None
        lane_change_finish_time = None
        is_jittering = False
        feature_seq_len = len(feature_sequence)
        prev_timestamp = -1.0
        obs_actual_lane_ids = []
        obs_traj = []
        total_observed_time_span = 0.0

        # This goes through all the subsequent features in this sequence
        # of features up to the maximum_observation_time.
        for j in range(idx_curr, feature_seq_len):
            # If timespan exceeds max. observation time, then end observing.
            time_span = feature_sequence[j].timestamp - feature_curr.timestamp
            if time_span > param_fea['maximum_observation_time']:
                break
            total_observed_time_span = time_span

            #####################################################################
            # Update the obstacle trajectory:
            # Only update for consecutive (sampling rate = 0.1sec) points.
            obs_traj.append((feature_sequence[j].position.x,
                             feature_sequence[j].position.y,
                             feature_sequence[j].velocity_heading,
                             feature_sequence[j].speed,
                             feature_sequence[j].length,
                             feature_sequence[j].width,
                             feature_sequence[j].timestamp))

            #####################################################################
            # Update the lane-change info (mainly for cruise scenario):
            if feature_sequence[j].HasField('lane') and \
               feature_sequence[j].lane.HasField('lane_feature'):
                # If jittering or done, then jump over this part.
                if (is_jittering or has_finished_lane_change):
                    continue
                # Record the sequence of lane_segments the obstacle stepped on.
                lane_id_j = feature_sequence[j].lane.lane_feature.lane_id
                if lane_id_j not in obs_actual_lane_ids:
                    obs_actual_lane_ids.append(lane_id_j)
                # If step into another lane, label lane change to be started.
                if lane_id_j not in curr_lane_segments:
                    # If it's the first time, log new_lane_id
                    if not has_started_lane_change:
                        has_started_lane_change = True
                        lane_change_start_time = time_span
                        new_lane_id = lane_id_j
                else:
                    # If it stepped into other lanes and now comes back, it's jittering!
                    if has_started_lane_change:
                        is_jittering = True
                        continue
                # If roughly get to the center of another lane, label lane change to be finished.
                left_bound = feature_sequence[j].lane.lane_feature.dist_to_left_boundary
                right_bound = feature_sequence[j].lane.lane_feature.dist_to_right_boundary
                if left_bound / (left_bound + right_bound) > (0.5 - param_fea['lane_change_finish_condition']) and \
                   left_bound / (left_bound + right_bound) < (0.5 + param_fea['lane_change_finish_condition']):
                    if has_started_lane_change:
                        # This means that the obstacle has finished lane change.
                        has_finished_lane_change = True
                        lane_change_finish_time = time_span
                    else:
                        # This means that the obstacle moves back to the center
                        # of the original lane for the first time.
                        if lane_change_finish_time is None:
                            lane_change_finish_time = time_span

        if len(obs_actual_lane_ids) == 0:
            # print("No lane id")
            return

        # Update the observation_dict:
        dict_val = dict()
        dict_val['obs_traj'] = obs_traj
        dict_val['obs_traj_len'] = len(obs_traj)
        dict_val['obs_actual_lane_ids'] = obs_actual_lane_ids
        dict_val['has_started_lane_change'] = has_started_lane_change
        dict_val['has_finished_lane_change'] = has_finished_lane_change
        dict_val['lane_change_start_time'] = lane_change_start_time
        dict_val['lane_change_finish_time'] = lane_change_finish_time
        dict_val['is_jittering'] = is_jittering
        dict_val['new_lane_id'] = new_lane_id
        dict_val['total_observed_time_span'] = total_observed_time_span
        self.observation_dict["{}@{:.3f}".format(
            feature_curr.id, feature_curr.timestamp)] = dict_val
        return

    '''
    @brief Based on the observation, label each lane sequence accordingly:
              - label whether the obstacle is on the lane_sequence
                within a certain amount of time.
              - if there is lane chage, label the time it takes to get to that lane.
    '''

    def LabelSingleLane(self, period_of_interest=3.0):
        output_features = offline_features_pb2.Features()
        for obs_id, feature_sequence in self.feature_dict.items():
            feature_seq_len = len(feature_sequence)
            for idx, feature in enumerate(feature_sequence):
                if not feature.HasField('lane') or \
                   not feature.lane.HasField('lane_feature'):
                    # print "No lane feature, cancel labeling"
                    continue

                # Observe the subsequent Features
                if "{}@{:.3f}".format(feature.id, feature.timestamp) not in self.observation_dict:
                    continue
                observed_val = self.observation_dict["{}@{:.3f}".format(
                    feature.id, feature.timestamp)]

                lane_sequence_dict = dict()
                # Based on the observation, label data.
                for lane_sequence in feature.lane.lane_graph.lane_sequence:
                    # Sanity check.
                    if len(lane_sequence.lane_segment) == 0:
                        print('There is no lane segment in this sequence.')
                        continue

                    # Handle jittering data
                    if observed_val['is_jittering']:
                        lane_sequence.label = -10
                        lane_sequence.time_to_lane_center = -1.0
                        lane_sequence.time_to_lane_edge = -1.0
                        continue

                    # Handle the case that we didn't obesrve enough Features to label
                    if observed_val['total_observed_time_span'] < period_of_interest and \
                       not observed_val['has_started_lane_change']:
                        lane_sequence.label = -20
                        lane_sequence.time_to_lane_center = -1.0
                        lane_sequence.time_to_lane_edge = -1.0

                    # The current lane is obstacle's original lane. (labels: 0,2,4)
                    if lane_sequence.vehicle_on_lane:
                        # Obs is following ONE OF its original lanes:
                        if not observed_val['has_started_lane_change'] or \
                           observed_val['lane_change_start_time'] > period_of_interest:
                            # Record this lane_sequence's lane_ids
                            current_lane_ids = []
                            for k in range(len(lane_sequence.lane_segment)):
                                if lane_sequence.lane_segment[k].HasField('lane_id'):
                                    current_lane_ids.append(lane_sequence.lane_segment[k].lane_id)

                            is_following_this_lane = True
                            for l_id in range(1, min(len(current_lane_ids),
                                                     len(observed_val['obs_actual_lane_ids']))):
                                if current_lane_ids[l_id] != observed_val['obs_actual_lane_ids'][l_id]:
                                    is_following_this_lane = False
                                    break

                            # Obs is following this original lane:
                            if is_following_this_lane:
                                # Obstacle is following this original lane and moved to lane-center
                                if observed_val['lane_change_finish_time'] is not None:
                                    lane_sequence.label = 4
                                    lane_sequence.time_to_lane_edge = -1.0
                                    lane_sequence.time_to_lane_center = -1.0
                                # Obstacle is following this original lane but is never at lane-center:
                                else:
                                    lane_sequence.label = 2
                                    lane_sequence.time_to_lane_edge = -1.0
                                    lane_sequence.time_to_lane_center = -1.0
                            # Obs is following another original lane:
                            else:
                                lane_sequence.label = 0
                                lane_sequence.time_to_lane_edge = -1.0
                                lane_sequence.time_to_lane_center = -1.0

                        # Obs has stepped out of this lane within period_of_interest.
                        else:
                            lane_sequence.label = 0
                            lane_sequence.time_to_lane_edge = -1.0
                            lane_sequence.time_to_lane_center = -1.0

                    # The current lane is NOT obstacle's original lane. (labels: -1,1,3)
                    else:
                        # Obstacle is following the original lane.
                        if not observed_val['has_started_lane_change'] or \
                           observed_val['lane_change_start_time'] > period_of_interest:
                            lane_sequence.label = -1
                            lane_sequence.time_to_lane_edge = -1.0
                            lane_sequence.time_to_lane_center = -1.0
                        else:
                            new_lane_id_is_in_this_lane_seq = False
                            for lane_segment in lane_sequence.lane_segment:
                                if lane_segment.lane_id == observed_val['new_lane_id']:
                                    new_lane_id_is_in_this_lane_seq = True
                                    break
                            # Obstacle has changed to this lane.
                            if new_lane_id_is_in_this_lane_seq:
                                # Obstacle has finished lane changing within time_of_interest.
                                if observed_val['has_finished_lane_change'] and \
                                   observed_val['lane_change_finish_time'] < period_of_interest:
                                    lane_sequence.label = 3
                                    lane_sequence.time_to_lane_edge = observed_val['lane_change_start_time']
                                    lane_sequence.time_to_lane_center = observed_val['lane_change_finish_time']
                                # Obstacle started lane changing but haven't finished yet.
                                else:
                                    lane_sequence.label = 1
                                    lane_sequence.time_to_lane_edge = observed_val['lane_change_start_time']
                                    lane_sequence.time_to_lane_center = -1.0

                            # Obstacle has changed to some other lane.
                            else:
                                lane_sequence.label = -1
                                lane_sequence.time_to_lane_edge = -1.0
                                lane_sequence.time_to_lane_center = -1.0

                for lane_sequence in feature.lane.lane_graph.lane_sequence:
                    lane_sequence_dict[lane_sequence.lane_sequence_id] = [lane_sequence.label,
                                                                          lane_sequence.time_to_lane_center, lane_sequence.time_to_lane_edge]
                self.cruise_label_dict["{}@{:.3f}".format(
                    feature.id, feature.timestamp)] = lane_sequence_dict
        np.save(self.filepath + '.cruise_label.npy', self.cruise_label_dict)

    def LabelTrajectory(self, period_of_interest=3.0):
        output_features = offline_features_pb2.Features()
        for obs_id, feature_sequence in self.feature_dict.items():
            for idx, feature in enumerate(feature_sequence):
                # Observe the subsequent Features
                if "{}@{:.3f}".format(feature.id, feature.timestamp) not in self.observation_dict:
                    continue
                observed_val = self.observation_dict["{}@{:.3f}".format(
                    feature.id, feature.timestamp)]
                self.future_status_dict["{}@{:.3f}".format(
                    feature.id, feature.timestamp)] = observed_val['obs_traj']
        np.save(self.filepath + '.future_status.npy', self.future_status_dict)
        #         for point in observed_val['obs_traj']:
        #             traj_point = feature.future_trajectory_points.add()
        #             traj_point.path_point.x = point[0]
        #             traj_point.path_point.y = point[1]
        #             traj_point.path_point.velocity_heading = point[2]
        #             traj_point.timestamp = point[3]

        #         output_features.feature.add().CopyFrom(feature)

        # self.SaveOutputPB(self.filepath + '.future_status.label', output_features)

    def LabelJunctionExit(self):
        '''
        label feature trajectory according to real future lane sequence in 7s
        '''
        output_features = offline_features_pb2.Features()
        for obs_id, feature_sequence in self.feature_dict.items():
            feature_seq_len = len(feature_sequence)
            for i, fea in enumerate(feature_sequence):
                # Sanity check.
                if not fea.HasField('junction_feature') or \
                   not len(fea.junction_feature.junction_exit):
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
                mask = [0] * 12
                for junction_exit in fea.junction_feature.junction_exit:
                    if junction_exit.HasField('exit_lane_id'):
                        exit_dict[junction_exit.exit_lane_id] = \
                            BoundingRectangle(junction_exit.exit_position.x,
                                              junction_exit.exit_position.y,
                                              junction_exit.exit_heading,
                                              0.01,
                                              junction_exit.exit_width)
                        exit_pos = np.array([junction_exit.exit_position.x,
                                             junction_exit.exit_position.y])
                        exit_pos_dict[junction_exit.exit_lane_id] = exit_pos
                        delta_pos = exit_pos - curr_pos
                        angle = math.atan2(delta_pos[1], delta_pos[0]) - heading
                        d_idx = int((angle / (2.0 * np.pi) + 1.0 / 24) * 12 % 12)
                        mask[d_idx] = 1

                # Searching for up to 100 frames (10 seconds)
                for j in range(i, min(i + 100, feature_seq_len)):
                    car_bounding = BoundingRectangle(feature_sequence[j].position.x,
                                                     feature_sequence[j].position.y,
                                                     math.atan2(feature_sequence[j].raw_velocity.y,
                                                                feature_sequence[j].raw_velocity.x),
                                                     feature_sequence[j].length,
                                                     feature_sequence[j].width)
                    for key, value in exit_dict.items():
                        if car_bounding.overlap(value):
                            exit_pos = exit_pos_dict[key]
                            delta_pos = exit_pos - curr_pos
                            angle = math.atan2(
                                delta_pos[1], delta_pos[0]) - heading
                            d_idx = int((angle / (2.0 * np.pi) + 1.0 / 24) * 12 % 12)
                            label = [0] * 12
                            label[d_idx] = 1
                            fea.junction_feature.junction_mlp_label.extend(label)
                            self.junction_label_dict["{}@{:.3f}".format(
                                fea.id, fea.timestamp)] = label + mask
                            break  # actually break two level
                    else:
                        continue
                    break
        np.save(self.filepath + '.junction_label.npy', self.junction_label_dict)
        #         if fea.HasField('junction_feature') and \
        #            len(fea.junction_feature.junction_mlp_label) > 0:
        #             output_features.feature.add().CopyFrom(fea)

        # self.SaveOutputPB(self.filepath + '.junction.label', output_features)

    def Label(self):
        self.LabelTrajectory()
        self.LabelSingleLane()
        self.LabelJunctionExit()
        # TODO(all):
        #   - implement label multiple lane
