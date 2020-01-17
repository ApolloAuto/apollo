#!/usr/bin/env python3

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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


import h5py
import numpy as np
import os

from modules.prediction.proto import offline_features_pb2


junction_label_label_dim = 12
future_status_label_dim = 30

'''
Read a single dataforlearn.bin file and output a list of DataForLearning
that is contained in that file.
'''


def LoadDataForLearning(filepath):
    list_of_data_for_learning = \
        offline_features_pb2.ListDataForLearning()
    with open(filepath, 'rb') as file_in:
        list_of_data_for_learning.ParseFromString(file_in.read())
    return list_of_data_for_learning.data_for_learning


'''
Read a single .npy dictionary file and get its content.
'''


def LoadLabels(filepath):
    mydict = np.load(filepath).item()
    return mydict


'''
Merge two dictionary into a single one and return.
'''


def MergeTwoDicts(dict1, dict2):
    newdict = dict1.copy()
    newdict.update(dict2)
    return newdict


'''
Merge all dictionaries directly under a directory
'''


def MergeDicts(dirpath, dict_name='future_status'):
    list_of_files = os.listdir(dirpath)
    dict_merged = None

    for file in list_of_files:
        full_path = os.path.join(dirpath, file)
        if file.split('.')[-1] == 'npy' and file.split('.')[-2] == dict_name:
            dict_curr = LoadLabels(full_path)
            if dict_merged is None:
                dict_merged = dict_curr.copy()
            else:
                dict_merged.update(dict_curr)

    np.save(dirpath + '/' + dict_name + '.npy', dict_merged)
    return dict_merged


'''
Go through every entry of data_for_learn proto and get the corresponding labels.
Save the output file into h5 format (array of lists with each list being a data
point for training/validating).
'''


def CombineFeaturesAndLabels(feature_path, label_path, dict_name='future_status'):
    list_of_data_for_learning = LoadDataForLearning(feature_path)
    dict_labels = LoadLabels(label_path)

    output_np_array = []
    for data_for_learning in list_of_data_for_learning:
        # features_for_learning: list of doubles
        features_for_learning = list(data_for_learning.features_for_learning)
        key = "{}@{:.3f}".format(data_for_learning.id, data_for_learning.timestamp)

        # Sanity checks to see if this data-point is valid or not.
        if key not in dict_labels:
            print('Cannot find a feature-to-label mapping.')
            continue

        labels = None
        list_curr = None
        if dict_name == 'junction_label':
            if len(dict_labels[key]) != junction_label_label_dim:
                continue
            labels = dict_labels[key]
            list_curr = features_for_learning + labels
        elif dict_name == 'future_status':
            if len(dict_labels[key]) < future_status_label_dim:
                continue
            labels = dict_labels[key][:30]
            list_curr = [len(features_for_learning)] + \
                features_for_learning + labels

        output_np_array.append(list_curr)

    output_np_array = np.array(output_np_array)

    np.save(feature_path + '.features+' + dict_name + '.npy', output_np_array)


'''
Merge all files of features+labels into a single one
'''


def MergeCombinedFeaturesAndLabels(dirpath):
    list_of_files = os.listdir(dirpath)

    features_labels_merged = []
    for file in list_of_files:
        full_path = os.path.join(dirpath, file)
        if file.split('.')[-1] == 'npy' and \
           file.split('.')[-2] == 'labels' and \
           file.split('.')[0] == 'datalearn':
            features_labels_curr = np.load(full_path).tolist()
            features_labels_merged += features_labels_curr

    np.save(dirpath + '/training_data.npy', np.array(features_labels_merged))


'''
It takes terminal folder as input, then
1. Merge all label dicts.
2. Go through every data_for_learn proto, and find the corresponding label
3. Merge all features+labels files into a single one: data.npy
'''


def PrepareDataForTraining(dirpath):
    MergeDicts(dirpath)

    list_of_files = os.listdir(dirpath)
    for file in list_of_files:
        full_path = os.path.join(dirpath, file)
        if file.split('.')[-1] == 'bin' and \
           file.split('.')[0] == 'datalearn':
            CombineFeaturesAndLabels(full_path, dirpath + 'labels.npy')

    MergeCombinedFeaturesAndLabels(dirpath)
