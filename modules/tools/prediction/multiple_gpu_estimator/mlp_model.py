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

# Copyright 2017 The TensorFlow Authors. All Rights Reserved.
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

import modules.tools.prediction.multiple_gpu_estimator.model_base


dim_input = 62
dim_hidden_1 = 30
dim_hidden_2 = 15
dim_output = 4


class MlpModel(model_base.ModelBase):
    """prediction model with fully connected layers."""

    def __init__(self,
                 is_training=True,
                 batch_norm_decay=0.999,
                 batch_norm_epsilon=0.001,
                 data_format='channels_last'):
        super(MlpModel, self).__init__(is_training, data_format,
                                       batch_norm_decay, batch_norm_epsilon)

    def forward_pass(self, x, input_data_format='channels_last'):
        """Build the core model within the graph."""
        x = self._fully_connected_with_bn(
            x,
            dim_input,
            kernel_initializer=tf.contrib.keras.initializers.he_normal(),
            kernel_regularizer=tf.contrib.layers.l2_regularizer(0.01))
        x = self._fully_connected_with_bn(
            x,
            dim_hidden_1,
            kernel_initializer=tf.contrib.keras.initializers.he_normal(),
            kernel_regularizer=tf.contrib.layers.l2_regularizer(0.01))
        x = self._fully_connected_with_bn(
            x,
            dim_hidden_2,
            kernel_initializer=tf.contrib.keras.initializers.he_normal(),
            kernel_regularizer=tf.contrib.layers.l2_regularizer(0.01))
        x = self._fully_connected(x, dim_output)
        return x

    def _fully_connected_with_bn(self,
                                 x,
                                 out_dim,
                                 kernel_initializer=None,
                                 kernel_regularizer=None):
        x = self._fully_connected(
            x,
            out_dim,
            kernel_initializer=kernel_initializer,
            kernel_regularizer=kernel_regularizer)
        x = self._relu(x)
        x = self._batch_norm(x)
        return x
