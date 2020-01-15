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

import argparse
import logging
import os

from sklearn.model_selection import train_test_split
from sklearn.utils import class_weight
from torch.autograd import Variable
from torch.utils.data import Dataset, DataLoader, sampler
import h5py
import numpy as np
import sklearn
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

from common.configure import parameters
from proto.cruise_model_pb2 import TensorParameter, InputParameter,\
    Conv1dParameter, DenseParameter, ActivationParameter, MaxPool1dParameter,\
    AvgPool1dParameter, LaneFeatureConvParameter, ObsFeatureFCParameter,\
    ClassifyParameter, RegressParameter, CruiseModelParameter


"""
@requirement:
    pytorch 0.4.1
"""

'''
This file includes all model definitions and related loss functions.
'''


'''
Model details:
    - Fully-connected layers for classification and regression, respectively.
    - It will compute a classification score indicating the probability
      of the obstacle choosing the given lane.
    - It will also compute a time indicating how soon the obstacle will reach
      the center of the given lane.
'''


class FullyConn_NN(torch.nn.Module):
    def __init__(self):
        super(FullyConn_NN, self).__init__()
        self.classify = torch.nn.Sequential(
            nn.Linear(174, 88),
            nn.Sigmoid(),
            nn.Dropout(0.3),

            nn.Linear(88, 55),
            nn.Sigmoid(),
            nn.Dropout(0.2),

            nn.Linear(55, 23),
            nn.Sigmoid(),
            nn.Dropout(0.3),

            nn.Linear(23, 10),
            nn.Sigmoid(),
            nn.Dropout(0.0),

            nn.Linear(10, 1),
            nn.Sigmoid()
        )
        self.regress = torch.nn.Sequential(
            nn.Linear(174, 88),
            nn.ReLU(),
            nn.Dropout(0.1),

            nn.Linear(88, 23),
            nn.ReLU(),
            nn.Dropout(0.1),

            nn.Linear(23, 1),
            nn.ReLU()
        )

    def forward(self, x):
        out_c = self.classify(x)
        out_r = self.regress(x)
        return out_c, out_r


class FCNN_CNN1D(torch.nn.Module):
    def __init__(self):
        super(FCNN_CNN1D, self).__init__()
        self.lane_feature_conv = torch.nn.Sequential(
            nn.Conv1d(4, 10, 3, stride=1),\
            # nn.BatchNorm1d(10),\
            nn.ReLU(),\
            #nn.Conv1d(10, 16, 3, stride=2),\
            # nn.BatchNorm1d(16),\
            # nn.ReLU(),\
            nn.Conv1d(10, 25, 3, stride=2),\
            # nn.BatchNorm1d(25)
        )
        self.lane_feature_maxpool = nn.MaxPool1d(4)
        self.lane_feature_avgpool = nn.AvgPool1d(4)
        self.lane_feature_dropout = nn.Dropout(0.0)

        self.obs_feature_fc = torch.nn.Sequential(
            nn.Linear(68, 40),
            nn.Sigmoid(),
            nn.Dropout(0.0),
            nn.Linear(40, 24),
            nn.Sigmoid(),
            nn.Dropout(0.0),
        )

        self.classify = torch.nn.Sequential(
            nn.Linear(124, 66),
            nn.Sigmoid(),
            nn.Dropout(0.3),

            nn.Linear(66, 48),
            nn.Sigmoid(),
            nn.Dropout(0.1),

            nn.Linear(48, 11),
            nn.Sigmoid(),
            nn.Dropout(0.1),

            nn.Linear(11, 1),\
            # nn.Sigmoid()
        )
        self.regress = torch.nn.Sequential(
            nn.Linear(125, 77),
            nn.ReLU(),
            nn.Dropout(0.2),

            nn.Linear(77, 46),
            nn.ReLU(),
            nn.Dropout(0.2),

            nn.Linear(46, 12),
            nn.ReLU(),
            nn.Dropout(0.1),

            nn.Linear(12, 1),
            nn.ReLU()
        )

    def forward(self, x):
        lane_fea = x[:, -80:]
        lane_fea = lane_fea.view(lane_fea.size(0), 4, 20)

        obs_fea = x[:, :-80]

        lane_fea = self.lane_feature_conv(lane_fea)

        lane_fea_max = self.lane_feature_maxpool(lane_fea)
        lane_fea_avg = self.lane_feature_avgpool(lane_fea)

        lane_fea = torch.cat([lane_fea_max.view(lane_fea_max.size(0), -1),
                              lane_fea_avg.view(lane_fea_avg.size(0), -1)], 1)
        lane_fea = self.lane_feature_dropout(lane_fea)

        obs_fea = self.obs_feature_fc(obs_fea)

        tot_fea = torch.cat([lane_fea, obs_fea], 1)
        out_c = self.classify(tot_fea)
        out_r = self.regress(torch.cat([tot_fea, out_c], 1))

        return out_c, out_r
