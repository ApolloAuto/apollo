#!/usr/bin/env python

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
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
This module creates steering model .
"""
import tensorflow as tf
from keras.models import Model
from keras.layers import Dense
from keras.layers import Input
from keras.layers import Dropout
from keras.layers import Flatten
from keras.layers import Lambda
from keras.layers import ELU
from keras.layers import GaussianNoise
from keras.layers.convolutional import Conv2D
from keras.optimizers import Nadam


def get_model(input_shape):
    """

    :param input_shape: the input shape
    :return: model
    """

    std = 1.0 / 255 * 0.1
    output_shape = (320, 320, 3)
    inputs = Input(shape=input_shape, name='input')
    x = Lambda(tf.image.rgb_to_hsv, input_shape=input_shape, output_shape=output_shape)(inputs)
    x = GaussianNoise(std, input_shape=input_shape)(x)

    x = Conv2D(24, (5, 5), strides=(2, 2), activation='elu', padding='valid')(x)
    x = Conv2D(24, (5, 5), strides=(2, 2), activation='elu', padding='valid')(x)
    x = Dropout(0.2)(x)
    x = Conv2D(48, (3, 3), strides=(1, 1), activation='elu', padding='valid')(x)
    x = Conv2D(48, (3, 3), strides=(2, 2), activation='elu', padding='valid')(x)
    x = Dropout(0.2)(x)
    x = Conv2D(64, (3, 3), strides=(1, 1), activation='elu', padding='valid')(x)
    x = Conv2D(64, (3, 3), strides=(2, 2), activation='elu', padding='valid')(x)
    x = Dropout(0.2)(x)
    x = Conv2D(96, (3, 3), strides=(1, 1), activation='elu', padding='valid')(x)
    x = Conv2D(96, (3, 3), strides=(1, 1), activation='elu', padding='valid')(x)
    x = Dropout(0.2)(x)
    x = Flatten()(x)

    x = Dense(1024)(x)
    x = ELU()(x)
    x = Dropout(.2)(x)
    x = Dense(256)(x)

    x = ELU()(x)
    x = Dropout(.2)(x)
    x = Dense(32)(x)
    x = ELU()(x)
    x = Dense(1)(x)

    model = Model(inputs=[inputs], outputs=[x])
    nadam = Nadam(lr=0.0001, schedule_decay=0.02)
    model.compile(optimizer=nadam, loss="mse")

    return model