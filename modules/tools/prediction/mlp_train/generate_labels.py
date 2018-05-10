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

from common.configure import parameters
from common.feature_io import load_protobuf
from common.feature_io import save_protobuf
from common.feature_io import build_trajectory
from common.trajectory import TrajectoryToSample


def label_file(input_file):
    """
    label each feature file
    """
    file_name, file_ext = os.path.splitext(input_file)
    output_file = file_name + ".label" + file_ext

    # read input file and save them in dict
    features = load_protobuf(input_file)

    # for each obstacle ID, sort dict by their timestamp
    fea_trajs = build_trajectory(features)

    # for each obstacle ID, label them, remove record cannot be labeled
    for fea_key, fea_traj in fea_trajs.items():
        fea_traj = fea_trajs[fea_key]
        fea_traj = TrajectoryToSample.clean(fea_traj)
        fea_traj = TrajectoryToSample.label(fea_traj)
        for i, fea in enumerate(fea_traj):
            if not fea.HasField('label_update_time_delta'):
                del fea_traj[i]
                continue
            if fea.label_update_time_delta < parameters['feature']['threshold_label_time_delta']:
                del fea_traj[i]
        fea_trajs[fea_key] = fea_traj
    # save them in the output file with the same format as the input file
    save_protobuf(output_file, fea_trajs.values())


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description = 'Generate labels')
    parser.add_argument('-d', '--dir', help = 'directory containing features')
    parser.add_argument('-f', '--file', help = 'feature file')

    args = parser.parse_args()
    directory = args.dir
    file = args.file

    if directory and os.path.isdir(directory):
        for feature_file in glob.glob(directory + '/*.bin'):
            print "Processing feature file: ", feature_file
            label_file(feature_file)

    if file and os.path.isfile(file):
        print "Processing feature file: ", file
        label_file(file)
