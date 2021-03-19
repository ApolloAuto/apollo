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

"""Prediction data set"""

import os

import tensorflow as tf


dim_input = 62


class MlpDataSet(object):
    """Prediction mlp data set.

    """

    def __init__(self, data_dir, subset='train'):
        self.data_dir = data_dir
        self.subset = subset

    def get_filenames(self):
        if self.subset in ['train', 'validation', 'eval']:
            return [os.path.join(self.data_dir, self.subset + '.tfrecords')]
        else:
            raise ValueError('Invalid data subset "%s"' % self.subset)

    def parser(self, serialized_example):
        """Parses a single tf.Example into image and label tensors."""
        # Dimensions of the images in the CIFAR-10 dataset.

        features = tf.parse_single_example(
            serialized_example,
            features={
                'data': tf.FixedLenFeature([62], tf.float32),
                'label': tf.FixedLenFeature([1], tf.float32),
            })

        image = features['data']
        label = tf.cast(features['label'], tf.int32)+1
        return image, label

    def make_batch(self, batch_size):
        """Read the images and labels from 'filenames'."""
        filenames = self.get_filenames()
        # Repeat infinitely.
        dataset = tf.data.TFRecordDataset(filenames).repeat()

        # Parse records.
        dataset = dataset.map(self.parser, num_parallel_calls=batch_size)

        # Potentially shuffle records.
        if self.subset == 'train':
            min_queue_examples = int(
                MlpDataSet.num_examples_per_epoch(self.subset) * 0.1)
            # Ensure that the capacity is sufficiently large to provide good random
            # shuffling.
            dataset = dataset.shuffle(buffer_size=min_queue_examples +
                                      3 * batch_size)

        # Batch it up.
        dataset = dataset.batch(batch_size)
        iterator = dataset.make_one_shot_iterator()
        image_batch, label_batch = iterator.get_next()

        return image_batch, label_batch

    @staticmethod
    def num_examples_per_epoch(subset='train'):
        if subset == 'train':
            return 13000000
        elif subset == 'validation':
            return 1600000
        elif subset == 'eval':
            return 1600000
        else:
            raise ValueError('Invalid data subset "%s"' % subset)
