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
This module creates acceleration model .
"""
from keras.models import Sequential
from keras.layers.core import Dense
from keras.layers.core import Flatten
from keras.layers.convolutional_recurrent import ConvLSTM2D
from keras.layers.normalization import BatchNormalization
from keras.optimizers import RMSprop


def get_model(shape=None):
    """
    :param shape: the input shape
    :return: model sequential
    """
    seq = Sequential()
    seq.add(ConvLSTM2D(nb_filter=24, nb_row=5, nb_col=5,
                       input_shape=shape,
                       subsample=(4, 4),
                       border_mode='same', return_sequences=True))
    seq.add(BatchNormalization())

    seq.add(ConvLSTM2D(nb_filter=48, nb_row=3, nb_col=3,
                       subsample=(2, 2),
                       border_mode='same', return_sequences=True))
    seq.add(BatchNormalization())

    seq.add(ConvLSTM2D(nb_filter=64, nb_row=3, nb_col=3,
                       subsample=(2, 2),
                       border_mode='same', return_sequences=False))

    seq.add(Flatten())
    seq.add(Dense(512, activation='relu'))
    seq.add(Dense(1))

    seq.compile(loss='mse', optimizer=RMSprop(lr=0.00001))

    return seq