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
"""
@requirement:
    tensorflow 1.11
"""

import os
import h5py
import logging
import argparse
import numpy as np
import tensorflow as tf

dim_input = 3+60
dim_output = 12

def load_data(filename):
    """
    Load the data from h5 file to the format of numpy
    """
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
    print("load file success")
    return samples['data']

def data_preprocessing(data):
    X = data[:, :dim_input]
    Y = data[:, -dim_output:]
    return X, Y

if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description=
        'train neural network based on feature files and save parameters')
    parser.add_argument('filename', type=str, help='h5 file of data.')
    args = parser.parse_args()
    file = args.filename
    # load_data
    train_data = load_data(file)
    print("Data load success, with data shape: " + str(train_data.shape))
    X_train, Y_train = data_preprocessing(train_data)
    model = tf.keras.models.Sequential([
        tf.keras.layers.Dense(60, activation=tf.nn.relu),
        tf.keras.layers.Dropout(0.2),
        tf.keras.layers.Dense(30, activation=tf.nn.relu),
        tf.keras.layers.Dropout(0.2),
        tf.keras.layers.Dense(12, activation=tf.nn.softmax)])
    model.compile(optimizer='adam',
                  loss='categorical_crossentropy',
                  # loss='MSE',
                  metrics=['accuracy'])
    model.fit(X_train, Y_train, epochs=5)
