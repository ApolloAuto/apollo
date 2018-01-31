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

from configure import parameters
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
        clean the feature point when lane_id changing abruptly
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
                print trajectory[i]
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
                    continue;

                lane_id_j = trajectory[j].lane.lane_feature.lane_id
                trajectory[i].label_update_time_delta = time_span
                if lane_id_j in future_lane_ids:
                    continue
                else:
                    future_lane_ids.append(lane_id_j)

            if len(future_lane_ids) < 1:
                print "No lane id"
                continue
            print "Future lane ids = ", future_lane_ids

            seq_size = len(fea.lane.lane_graph.lane_sequence)
            for j in range(seq_size):
                seq = fea.lane.lane_graph.lane_sequence[j]
                if len(seq.lane_segment) == 0:
                    continue
                predict_lane_ids = []
                for k in range(len(seq.lane_segment)):
                    if seq.lane_segment[k].HasField('lane_id'):
                        predict_lane_ids.append(seq.lane_segment[k].lane_id)
                print "Predicted lane ids = ", predict_lane_ids

                seq.label = cls.cmp_lane_seq(future_lane_ids, predict_lane_ids)
                print "Label is set to be ", seq.label
        return trajectory

    @abc.abstractmethod
    def pack(self, trajectory):
        """ abstractmethod"""
        raise NotImplementedError
