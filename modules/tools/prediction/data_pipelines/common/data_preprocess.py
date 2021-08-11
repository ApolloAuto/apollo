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

import os
import h5py
import numpy as np
from random import choice
from random import randint
from random import shuffle
from time import time


def load_h5(filename):
    if not (os.path.exists(filename)):
        logging.error("file: {}, does not exist".format(filename))
        os._exit(1)
    if os.path.splitext(filename)[1] != '.h5':
        logging.error("file: {} is not an hdf5 file".format(filename))
        os._exit(1)

    samples = dict()
    h5_file = h5py.File(filename, 'r')
    for key in h5_file.keys():
        samples[key] = h5_file[key][:]

    return samples['data']


def down_sample(data):
    cutin_false_drate = 0.5
    go_false_drate = 0.8
    go_true_drate = 0.9
    cutin_true_drate = 0.0

    label = data[:, -1]
    size = np.shape(label)[0]

    cutin_false_index = (label == -1)
    go_false_index = (label == 0)
    go_true_index = (label == 1)
    cutin_true_index = (label == 2)

    rand = np.random.random((size))

    cutin_false_select = np.logical_and(cutin_false_index,
                                        rand > cutin_false_drate)
    cutin_true_select = np.logical_and(cutin_true_index,
                                       rand > cutin_true_drate)
    go_false_select = np.logical_and(go_false_index, rand > go_false_drate)
    go_true_select = np.logical_and(go_true_index, rand > go_true_drate)

    all_select = np.logical_or(cutin_false_select, cutin_true_select)
    all_select = np.logical_or(all_select, go_false_select)
    all_select = np.logical_or(all_select, go_true_select)

    data_downsampled = data[all_select, :]

    return data_downsampled


def train_test_split(data, train_rate):
    data_size = np.shape(data)[0]
    train_size = int(data_size * train_rate)
    train = data[0:train_size, ]
    test = data[train_size:, ]
    return train, test
