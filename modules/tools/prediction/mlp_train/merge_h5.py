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
import glob
import argparse
import datetime

import numpy as np
import h5py

def load_hdf5(filename):
    """
    load training samples from *.hdf5 file
    """
    if not(os.path.exists(filename)):
        print "file:", filename, "does not exist"
        os._exit(1)
    if os.path.splitext(filename)[1] != '.h5':
        print "file:", filename, "is not an hdf5 file"
        os._exit(1)

    h5_file = h5py.File(filename, 'r')
    values = h5_file.values()[0]
    print "load data size:", values.shape[0]
    return values


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'generate training samples\
            from a specified directory')
    parser.add_argument('directory', type=str,
            help='directory contains feature files in .h5')
    args = parser.parse_args()
    path = args.directory

    print "load h5 from directory:", format(path)
    if os.path.isdir(path):
        features = None
        labels = None

        h5_files = glob.glob(path + '/*.h5')
        print "Length of files:", len(h5_files)
        for i, h5_file in enumerate(h5_files):
            print "Process File", i, ":", h5_file
            feature = load_hdf5(h5_file)
            if np.any(np.isinf(feature)):
                print "inf data found"
            features = np.concatenate((features, feature), axis=0) if features is not None \
                    else feature
    else:
        print "Fail to find", path
        os._exit(-1)

    date = datetime.datetime.now().strftime('%Y-%m-%d')
    sample_dir = path + '/mlp_merge'
    if not os.path.exists(sample_dir):
        os.makedirs(sample_dir)
    sample_file = sample_dir + '/mlp_' + date + '.h5'
    print "Save samples file to:", sample_file
    h5_file = h5py.File(sample_file, 'w')
    h5_file.create_dataset('data', data=features)
    h5_file.close()

