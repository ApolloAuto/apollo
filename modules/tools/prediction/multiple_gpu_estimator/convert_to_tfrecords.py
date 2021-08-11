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

# Copyright 2015 The TensorFlow Authors. All Rights Reserved.
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
"""Converts MLP data to TFRecords file format with Example protos."""

import argparse
import os
import sys

import numpy as np
import tensorflow as tf


FLAGS = None
feature_dim = 62


def _float_feature(value):
    return tf.train.Feature(float_list=tf.train.FloatList(value=value))


def convert_to(bin_data, name):
    """Converts bin_data to tfrecords."""
    if bin_data.shape[0] % (feature_dim + 1) != 0:
        raise ValueError(
            'data size (%d) must be multiple of feature_dim + 1 (%d).' %
            (bin_data.shape[0], feature_dim + 1))
    num_examples = bin_data.shape[0] // (feature_dim + 1)
    print("num_examples:", num_examples)
    filename = os.path.join(name + '.tfrecords')
    print('Writing', filename)
    with tf.python_io.TFRecordWriter(filename) as writer:
        for index in range(0, num_examples):
            data_raw = bin_data[index * (feature_dim + 1):index *
                                (feature_dim + 1) + feature_dim]
            label_raw = np.array(
                [bin_data[index*(feature_dim + 1)+feature_dim]])
            example = tf.train.Example(
                features=tf.train.Features(
                    feature={
                        'data': _float_feature(data_raw),
                        'label': _float_feature(label_raw)
                    }))
            writer.write(example.SerializeToString())


def main(unused_argv):
    # Get the data.
    for path, subdirs, files in os.walk(FLAGS.directory):
        print("path:", path)
        print("subdirs:", subdirs)
        for name in files:
            filename = os.path.join(path, name)
            print("processing ", filename)
            bin_data = np.fromfile(filename, dtype=np.float32)

            # Convert to Examples and write the result to TFRecords.
            convert_to(bin_data, filename)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--directory',
        type=str,
        default='/tmp/data/prediction',
        help='Directory to download data files and write the converted result')
    FLAGS, unparsed = parser.parse_known_args()
    tf.app.run(main=main, argv=[sys.argv[0]] + unparsed)
