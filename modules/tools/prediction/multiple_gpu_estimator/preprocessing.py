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

import tensorflow as tf
import numpy as np
from glob import glob

feature_dim = 62
train_data = np.zeros([20000000, feature_dim+1], dtype=np.float32)
test_data = np.zeros([2000000, feature_dim+1], dtype=np.float32)
eval_data = np.zeros([2000000, feature_dim+1], dtype=np.float32)
train_idx, test_idx, eval_idx = 0, 0, 0
filenames = glob('/tmp/data/feature_v1_bin/*/*.label.bin')
for filename in filenames:
    print(filename)
    bin_data = np.fromfile(filename, dtype=np.float32)
    if bin_data.shape[0] % (feature_dim + 1) != 0:
        raise ValueError('data size (%d) must be multiple of feature_dim + 1 (%d).' %
                         (bin_data.shape[0], feature_dim + 1))
    num_examples = bin_data.shape[0] // (feature_dim + 1)
    for i in range(num_examples):
        label = int(bin_data[i*(feature_dim + 1)+feature_dim])
        data = bin_data[i*(feature_dim + 1):(i+1) *
                        (feature_dim + 1)].reshape([1, (feature_dim+1)])
        if label == 2:
            times = 17
            new_data = np.repeat(data, times, axis=0)
        elif label == 1:
            times = np.random.choice([2, 2, 2, 3, 3])
            new_data = np.repeat(data, times, axis=0)
        else:
            times = 1
            new_data = data

        if i % 10 == 8:
            test_data[test_idx:test_idx+times, :] = new_data
            test_idx += times
        elif i % 10 == 9:
            eval_data[eval_idx:eval_idx+times, :] = new_data
            eval_idx += times
        else:
            train_data[train_idx:train_idx+times, :] = new_data
            train_idx += times

train_data = train_data[:train_idx, :]
np.random.shuffle(train_data)
print(train_data.shape, train_idx)

test_data = test_data[:test_idx, :]
np.random.shuffle(test_data)
print(test_data.shape, test_idx)

eval_data = eval_data[:eval_idx, :]
np.random.shuffle(eval_data)
print(eval_data.shape, eval_idx)

# write to file
train_data[:13000000, :].tofile('/tmp/data/prediction/train.bin')
test_data[:1600000, :].tofile('/tmp/data/prediction/test.bin')
eval_data[:1600000, :].tofile('/tmp/data/prediction/eval.bin')
