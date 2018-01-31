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

import sys
import copy
import logging

from google.protobuf.internal import decoder
from google.protobuf.internal import encoder
import google.protobuf.text_format as text_format
from modules.prediction.proto import feature_pb2
from modules.prediction.proto import offline_features_pb2


def readVarint32(stream):
    """
    read block size from file stream
    """
    mask = 0x80 #(1 << 7)
    raw_varint32 = []
    while 1:
        b = stream.read(1)
        if b == "":
            break
        raw_varint32.append(b)
        if not (ord(b) & mask):
            break
    return raw_varint32


def load_protobuf(filename):
    """
    read a file in protobuf binary
    """
    features = []
    offline_features = offline_features_pb2.Features()
    with open(filename, 'rb') as file_in:
        offline_features.ParseFromString(file_in.read())
    for i in range(len(offline_features.feature)):
        features.append(offline_features.feature[i])

    return features


def load_label_feature(filename):
    features = []
    with open(filename, 'rb') as f:
        size = readVarint32(f)
        while size:
            read_bytes, _ = decoder._DecodeVarint32(size, 0)
            data = f.read(read_bytes)
            if len(data) < read_bytes:
                print "Fail to load protobuf"
                break
            fea = feature_pb2.Feature()
            fea.ParseFromString(data)
            features.append(fea)
            size = readVarint32(f)
    return features


def save_protobuf(filename, feature_trajectories):
    """
    save a features in the given filename
    """
    with open(filename, 'wb') as f:
        for features in feature_trajectories:
            for fea in features:
                serializedMessage = fea.SerializeToString()
                delimiter = encoder._VarintBytes(len(serializedMessage))
                f.write(delimiter + serializedMessage)

    f.close()


def build_trajectory(features):
    """
    classify features by id and build trajectories of feature
    """
    fea_dict = dict()
    for fea in features:
        if fea.id in fea_dict.keys():
            fea_dict[fea.id].append(fea)
        else:
            fea_dict[fea.id] = [fea]

    for k in fea_dict.keys():
        if len(fea_dict[k]) < 2:
            del fea_dict[k]
            continue
        fea_dict[k].sort(key = lambda x: x.timestamp)
    feature_trajectories = fea_dict

    return feature_trajectories
