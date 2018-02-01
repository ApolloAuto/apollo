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
                mlp_features = np.concatenate((mlp_features, mlp_feature_np),
                                              axis=0)
    mlp_features = mlp_features.reshape((np.shape(mlp_features)[0] / (mlp_feature_size + 1),
                                        (mlp_feature_size + 1)))
    print np.shape(mlp_features)
    return mlp_features


def generate_h5_file(filename):
    features = extract_mlp_features(filename)
    file_pre, file_ext = os.path.splitext(filename)
    output_file = file_pre + '.h5'
    h5_file = h5py.File(output_file, 'w')
    h5_file.create_dataset('data', data = features)
    h5_file.close()


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description = 'Generate H5 files')
    parser.add_argument('-d', '--dir', help = 'directory containing features')
    parser.add_argument('-f', '--file', help = 'feature file')

    args = parser.parse_args()
    directory = args.dir
    file = args.file

    if directory and os.path.isdir(directory):
        for feature_file in glob.glob(directory + '/*.label.bin'):
            print "Processing labeled feature file: ", feature_file
            generate_h5_file(feature_file)

    if file and os.path.isfile(file):
        print "Processing labeled feature file: ", file
        generate_h5_file(file)

