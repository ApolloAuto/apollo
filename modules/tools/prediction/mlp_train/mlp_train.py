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
    tensorflow- 1.3.0
    Keras-1.2.2
"""

import os
import h5py
import numpy as np
import logging
import argparse

import google.protobuf.text_format as text_format
from keras.callbacks import ModelCheckpoint
from keras.metrics import mse
from keras.models import Sequential, Model
from keras.layers.normalization import BatchNormalization
from keras.layers import Dense, Input
from keras.layers import Activation
from keras.layers import Dropout
from keras.utils import np_utils
from keras.regularizers import l2, l1
from sklearn.model_selection import train_test_split

import proto.fnn_model_pb2
from proto.fnn_model_pb2 import FnnModel, Layer

import common.log
from common.data_preprocess import load_h5
from common.data_preprocess import down_sample
from common.data_preprocess import train_test_split
from common.configure import parameters
from common.configure import labels


# Constants
dim_input = parameters['mlp']['dim_input']
dim_hidden_1 = parameters['mlp']['dim_hidden_1']
dim_hidden_2 = parameters['mlp']['dim_hidden_2']
dim_output = parameters['mlp']['dim_output']
train_data_rate = parameters['mlp']['train_data_rate']

evaluation_log_path = os.path.join(os.getcwd(), "evaluation_report")
common.log.init_log(evaluation_log_path, level=logging.DEBUG)


def load_data(filename):
    """
    Load the data from h5 file to the format of numpy
    """
    if not(os.path.exists(filename)):
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
    cutin_false_drate = 0.9
    go_false_drate = 0.9
    go_true_drate = 0.7
    cutin_true_drate = 0.0

    label = data[:, -1]
    size = np.shape(label)[0]

    cutin_false_index = (label == -1)
    go_false_index = (label == 0)
    go_true_index = (label == 1)
    cutin_true_index = (label == 2)

    rand = np.random.random((size))

    cutin_false_select = np.logical_and(cutin_false_index, rand > cutin_false_drate)
    cutin_true_select = np.logical_and(cutin_true_index, rand > cutin_true_drate)
    go_false_select = np.logical_and(go_false_index, rand > go_false_drate)
    go_true_select = np.logical_and(go_true_index, rand > go_true_drate)

    all_select = np.logical_or(cutin_false_select, cutin_true_select)
    all_select = np.logical_or(all_select, go_false_select)
    all_select = np.logical_or(all_select, go_true_select)

    data_downsampled = data[all_select, :]

    return data_downsampled


def get_param_norm(feature):
    """
    Normalize the samples and save normalized parameters
    """
    fea_mean = np.mean(feature, axis=0)
    fea_std = np.std(feature, axis=0) + 1e-6
    param_norm = (fea_mean, fea_std)
    return param_norm


def setup_model():
    """
    Set up neural network based on keras.Sequential
    """
    model = Sequential()
    model.add(Dense(dim_hidden_1,
        input_dim = dim_input,
        init = 'he_normal',
        activation = 'relu',
        W_regularizer = l2(0.01)))

    model.add(Dense(dim_hidden_2,
        init = 'he_normal',
        activation = 'relu',
        W_regularizer = l2(0.01)))

    model.add(Dense(dim_output,
        init='he_normal',
        activation = 'sigmoid',
        W_regularizer = l2(0.01)))

    model.compile(loss = 'binary_crossentropy',
                  optimizer = 'rmsprop',
                  metrics = ['accuracy'])

    return model


def evaluate_model(y, pred):
    """
    give the performance [recall, precision] of nn model

    Parameters
    ----------
    y: numpy.array;     real classess
    pred: numpy.array;  prediction classes

    Returns
    -------
    performance dict, store the performance in log file
    """
    y = y.reshape(-1)
    pred = pred.reshape(-1)

    go_true = (y == labels['go_true']).sum()
    go_false = (y == labels['go_false']).sum()
    index_go = np.logical_or(y == labels['go_false'], y == labels['go_true'])
    go_positive = (pred[index_go] == 1).sum()
    go_negative = (pred[index_go] == 0).sum()

    cutin_true = (y == labels['cutin_true']).sum()
    cutin_false = (y == labels['cutin_false']).sum()
    index_cutin = np.logical_or(y == labels['cutin_false'], y == labels['cutin_true'])
    cutin_positive = (pred[index_cutin] == 1).sum()
    cutin_negative = (pred[index_cutin] == 0).sum()

    logging.info("data size: {}, included:".format(y.shape[0]))
    logging.info("\t  True    False   Positive   Negative")
    logging.info(" Go:  {:7} {:7} {:7} {:7}".format(
            go_true, go_false, go_positive, go_negative))
    logging.info("Cutin:{:7} {:7} {:7} {:7}".format(
            cutin_true, cutin_false, cutin_positive, cutin_negative))

    logging.info("--------------------SCORE-----------------------------")
    logging.info("          recall   precision    F1-score")
    ctrue =  float(go_true + cutin_true)
    positive = float(go_positive + cutin_positive)
    tp = float((pred[y > 0.1] == 1).sum())
    recall = tp / ctrue if ctrue != 0 else 0.0
    precision = tp / positive if positive != 0 else 0.0
    fscore = 2 * precision * recall / (precision +
                recall) if precision + recall != 0 else 0.0
    logging.info("Positive:{:6.3}     {:6.3}     {:6.3}".format(recall, precision, fscore))

    go_tp = float((pred[y == 1] == 1).sum())
    go_recall = go_tp / go_true if go_true != 0 else 0.0
    go_precision = go_tp / go_positive if go_positive != 0 else 0.0
    go_fscore = 2 * go_precision * go_recall / (go_precision +
                go_recall) if go_precision + go_recall != 0 else 0.0
    logging.info("      Go:{:6.3}     {:6.3}     {:6.3}".format(go_recall, go_precision, go_fscore))

    cutin_tp = float((pred[y == 2] == 1).sum())
    cutin_recall = cutin_tp / cutin_true if cutin_true != 0 else 0.0
    cutin_precision = cutin_tp / cutin_positive if cutin_positive != 0 else 0.0
    cutin_fscore = 2 * cutin_precision * cutin_recall / (cutin_precision +
                cutin_recall) if cutin_precision + cutin_recall != 0 else 0.0
    logging.info("   Cutin:{:6.3}     {:6.3}     {:6.3}".format(
        cutin_recall, cutin_precision, cutin_fscore))
    logging.info("-----------------------------------------------------\n\n")

    performance = {'recall': [recall, go_recall, cutin_recall],
            'precision': [precision, go_precision, cutin_precision]}
    return performance


def save_model(model, param_norm, filename):
    """
    Save the trained model parameters into protobuf binary format file
    """
    net_params = FnnModel()
    net_params.samples_mean.columns.extend(param_norm[0].reshape(-1).tolist())
    net_params.samples_std.columns.extend(param_norm[1].reshape(-1).tolist())
    net_params.num_layer = 0
    for layer in model.flattened_layers:
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

    parser = argparse.ArgumentParser(description =
            'train neural network based on feature files and save parameters')
    parser.add_argument('filename', type = str, help = 'h5 file of data.')
    
    args = parser.parse_args()
    file = args.filename

    data = load_data(file)
    data = down_sample(data)
    print "Data load success."
    print "data size =", data.shape

    train_data, test_data = train_test_split(data, train_data_rate)

    print "training size =", train_data.shape

    X_train = train_data[:, 0 : dim_input]
    Y_train = train_data[:, -1]
    Y_trainc = Y_train > 0.1

    X_test = test_data[:, 0 : dim_input]
    Y_test = test_data[:, -1]
    Y_testc = Y_test > 0.1

    param_norm = get_param_norm(X_train)

    X_train = (X_train - param_norm[0]) / param_norm[1]

    X_test = (X_test - param_norm[0]) / param_norm[1]

    model = setup_model()

    model.fit(X_train, Y_trainc,
        shuffle = True,
        nb_epoch = 20,
        batch_size = 32)
    print "Model trained success."

    X_test = (X_test - param_norm[0]) / param_norm[1]

    score = model.evaluate(X_test, Y_testc)
    print "\nThe accuracy on testing dat is", score[1]

    logging.info("Test data loss: {}, accuracy: {} ".format(score[0], score[1]))
    Y_train_hat = model.predict_classes(X_train, batch_size = 32)
    Y_test_hat = model.predict_proba(X_test, batch_size = 32)
    logging.info("## Training Data:")
    evaluate_model(Y_train, Y_train_hat)
    for thres in [x / 100.0 for x in range(20, 80, 5)]:
        logging.info("##threshond = {} Testing Data:".format(thres))
        performance = evaluate_model(Y_test, Y_test_hat > thres)
    performance['accuracy'] = [score[1]]

    print "\nFor more detailed evaluation results, please refer to", \
          evaluation_log_path + ".log"
    
    model_path = os.path.join(os.getcwd(), "mlp_model.bin")
    save_model(model, param_norm, model_path)
    print "Model has been saved to", model_path
