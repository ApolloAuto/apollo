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
from . import proto.fnn_model_pb2
from .proto.fnn_model_pb2 import FnnModel, Layer
from sklearn.model_selection import train_test_split

dim_input = 7 + 72
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


def save_model(model, filename):
    """
    Save the trained model parameters into protobuf binary format file
    """
    net_params = FnnModel()
    net_params.num_layer = 0
    for layer in model.layers:
        net_params.num_layer += 1
        net_layer = net_params.layer.add()
        config = layer.get_config()
        net_layer.layer_input_dim = dim_input
        net_layer.layer_output_dim = dim_output
        if config['activation'] == 'relu':
            net_layer.layer_activation_func = proto.fnn_model_pb2.Layer.RELU
        elif config['activation'] == 'tanh':
            net_layer.layer_activation_func = proto.fnn_model_pb2.Layer.TANH
        elif config['activation'] == 'sigmoid':
            net_layer.layer_activation_func = proto.fnn_model_pb2.Layer.SIGMOID
        elif config['activation'] == 'softmax':
            net_layer.layer_activation_func = proto.fnn_model_pb2.Layer.SOFTMAX

        weights, bias = layer.get_weights()
        net_layer.layer_bias.columns.extend(bias.reshape(-1).tolist())
        for col in weights.tolist():
            row = net_layer.layer_input_weight.rows.add()
            row.columns.extend(col)
    net_params.dim_input = dim_input
    net_params.dim_output = dim_output
    with open(filename, 'wb') as params_file:
        params_file.write(net_params.SerializeToString())


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='train neural network based on feature files and save parameters')
    parser.add_argument('filename', type=str, help='h5 file of data.')
    args = parser.parse_args()
    file = args.filename
    # load_data
    data = load_data(file)
    print("Data load success, with data shape: " + str(data.shape))
    train_data, test_data = train_test_split(data, test_size=0.2)
    X_train, Y_train = data_preprocessing(train_data)
    X_test, Y_test = data_preprocessing(test_data)
    model = tf.keras.models.Sequential([
        tf.keras.layers.Dense(30, activation=tf.nn.relu),
        tf.keras.layers.Dense(20, activation=tf.nn.relu),
        tf.keras.layers.Dense(12, activation=tf.nn.softmax)])
    model.compile(optimizer='adam',
                  loss='categorical_crossentropy',
                  # loss='MSE',
                  metrics=['accuracy'])
    model.fit(X_train, Y_train, epochs=5)
    model_path = os.path.join(os.getcwd(), "junction_mlp_vehicle_model.bin")
    save_model(model, model_path)
    print("Model saved to: " + model_path)
    score = model.evaluate(X_test, Y_test)
    print("Testing accuracy is: " + str(score))
