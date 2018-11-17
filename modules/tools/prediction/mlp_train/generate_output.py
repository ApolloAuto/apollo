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

import os
import sys
import glob
import argparse
import logging
import numpy as np
import h5py

from common.configure import parameters
from common.feature_io import load_protobuf
from common.feature_io import load_label_feature
from common.feature_io import save_protobuf
from common.feature_io import build_trajectory
from common.trajectory import TrajectoryToSample

mlp_feature_size = parameters['mlp']['dim_input']


def extract_mlp_features(filename):
    features = load_label_feature(filename)

    mlp_features = None
    for feature in features:
        if not feature.HasField('lane') or \
           not feature.lane.HasField('lane_graph'):
            continue
        lane_seq_sz = len(feature.lane.lane_graph.lane_sequence)
        for i in range(lane_seq_sz):
            lane_seq = feature.lane.lane_graph.lane_sequence[i]
            if len(lane_seq.features.mlp_features) != mlp_feature_size:
                continue
            mlp_feature = []
            for i in range(mlp_feature_size):
                mlp_feature.append(lane_seq.features.mlp_features[i])
            mlp_feature.append(lane_seq.label)
            mlp_feature_np = np.array(mlp_feature)
            if mlp_features is None:
                mlp_features = mlp_feature_np
            else:
                mlp_features = np.concatenate(
                    (mlp_features, mlp_feature_np), axis=0)
    if (mlp_features is None) or (np.size(mlp_features) == 0):
        return
    mlp_features = mlp_features.reshape(
        (np.shape(mlp_features)[0] / (mlp_feature_size + 1),
         (mlp_feature_size + 1)))
    return mlp_features


def generate_output(filename, output_file):
    features = extract_mlp_features(filename)
    if (features is None) or (np.size(features) == 0):
        print("Failed to extract mlp features from {}".format(filename))
        return
    features = features.astype(np.float32)
    features.tofile(output_file)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate .bin files for tensorflow')
    parser.add_argument('input', type=str, help='input file')
    parser.add_argument('output', type=str, help='output file')
    args = parser.parse_args()
    print("Creating .bin: {} -> {}".format(args.input, args.output))
    if os.path.isfile(args.input):
        generate_output(args.input, args.output)
    else:
        print("{} is not a valid file.".format(args.input))
