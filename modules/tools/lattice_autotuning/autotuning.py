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
    Keras-2.1.3
"""

import os
import numpy as np
import logging
import argparse

from keras import layers
from keras import initializers
from keras import optimizers
from keras.models import Model
from keras.constraints import Constraint
import tensorflow as tf


class NonNegUnitNorm(Constraint):
    """
    keep weight with Unit Norm and non-negative
    """
    def __init__(self, axis=0):
        self.axis = axis
    def __call__(self, w):
        w *= tf.cast(tf.greater_equal(w, 0.0), tf.float32)
        return w / (1e-7 + tf.sqrt(tf.reduce_sum(tf.square(w), self.axis, True)))
    def get_config(self):
        return {'axis': self.axis}


def load_data(filename):
    # TODO load data from csv file to numpy
    return None


def preprocess(data):
    # TODO human costs minus planner costs
    return None


def dummy_loss(y_pred, delta):
    loss = tf.add(y_pred, delta)
    condition = tf.greater(loss, 0.0)
    loss = tf.where(condition, loss, tf.zeros_like(loss))
    return tf.reduce_mean(loss, axis=-1)


def setup_model():
    dim_feature = 4
    inp = layers.Input(shape=(dim_feature,))
    dense = layers.Dense(1,
            activation='linear',
            kernel_initializer=initializers.RandomUniform(minval=1.0, maxval=4.0),
            use_bias=False,
            kernel_constraint=NonNegUnitNorm())(inp)
    model = Model(inp, dense)
    sgd = optimizers.SGD(lr=1e-2, momentum=0.2, decay=5e-4)
    model.compile(optimizer=sgd, loss=dummy_loss)
    return model


def train_model(data, model):
    # train the model on data
    return None


if __name__ == "__main__":

    # parser = argparse.ArgumentParser(description =
    #         'train neural network based on feature files and save parameters')
    # parser.add_argument('filename', type = str, help = 'csv file of data.')
    
    # args = parser.parse_args()
    # file = args.filename

    # data = load_data(file)

    a = dummy_loss(np.array([1.0, 1.0, 1.0]), 0.1)
    print "a =", a

    model = setup_model()
    print "model =", model

