#!/usr/bin/env python3

###############################################################################
# Modification Copyright 2018 The Apollo Authors. All Rights Reserved.
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

# Copyright 2018 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

import tensorflow as tf


class ModelBase(object):
    def __init__(self,
                 is_training=True,
                 data_format='channels_last',
                 batch_norm_decay=0.999,
                 batch_norm_epsilon=0.001):
        """ModelBase constructor.

    Args:
      is_training: if build training or inference model.
      data_format: the data_format used during computation.
                   one of 'channels_first' or 'channels_last'.
    """
        self._batch_norm_decay = batch_norm_decay
        self._batch_norm_epsilon = batch_norm_epsilon
        self._is_training = is_training
        assert data_format in ('channels_first', 'channels_last')
        self._data_format = data_format

    def forward_pass(self, x):
        raise NotImplementedError(
            'forward_pass() is implemented in ResNet sub classes')

    def _conv(self, x, kernel_size, filters, strides, is_atrous=False):
        """Convolution."""

        padding = 'SAME'
        if not is_atrous and strides > 1:
            pad = kernel_size - 1
            pad_beg = pad // 2
            pad_end = pad - pad_beg
            if self._data_format == 'channels_first':
                x = tf.pad(
                    x,
                    [[0, 0], [0, 0], [pad_beg, pad_end], [pad_beg, pad_end]])
            else:
                x = tf.pad(
                    x,
                    [[0, 0], [pad_beg, pad_end], [pad_beg, pad_end], [0, 0]])
            padding = 'VALID'
        return tf.layers.conv2d(
            inputs=x,
            kernel_size=kernel_size,
            filters=filters,
            strides=strides,
            padding=padding,
            use_bias=False,
            data_format=self._data_format)

    def _batch_norm(self, x):
        if self._data_format == 'channels_first':
            data_format = 'NCHW'
        else:
            data_format = 'NHWC'
        return tf.contrib.layers.batch_norm(
            x,
            decay=self._batch_norm_decay,
            center=True,
            scale=True,
            epsilon=self._batch_norm_epsilon,
            is_training=self._is_training,
            fused=True,
            data_format=data_format)

    def _relu(self, x):
        return tf.nn.relu(x)

    def _fully_connected(self,
                         x,
                         out_dim,
                         kernel_initializer=None,
                         kernel_regularizer=None):
        with tf.name_scope('fully_connected') as name_scope:
            x = tf.layers.dense(
                x,
                out_dim,
                kernel_initializer=kernel_initializer,
                kernel_regularizer=kernel_regularizer)

        tf.logging.info('image after unit %s: %s', name_scope, x.get_shape())
        return x
