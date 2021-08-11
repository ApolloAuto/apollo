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

'''
This .py file includes functions for data preprocessing:
    - splitting data into two categories: go and cut-in for separate training.
    - balancing the datasets
'''

import argparse
import datetime
import os

import h5py
import numpy as np


def getListOfFiles(dirName):
    '''
    Given a directory (dirName), return a list containing the full-path
    of all files inside that directory (including all hierarchy).
    '''
    listOfFiles = os.listdir(dirName)
    allFiles = list()

    for entry in listOfFiles:
        fullPath = os.path.join(dirName, entry)
        if os.path.isdir(fullPath):
            allFiles = allFiles + getListOfFiles(fullPath)
        else:
            allFiles.append(fullPath)

    return allFiles


def load_hdf5(filename):
    """
    load training samples from *.hdf5 file
    """
    if not(os.path.exists(filename)):
        print("file:", filename, "does not exist")
        os._exit(1)
    if os.path.splitext(filename)[1] != '.h5':
        print("file:", filename, "is not an hdf5 file")
        os._exit(1)

    h5_file = h5py.File(filename, 'r')
    values = h5_file[list(h5_file.keys())[0]]
    #values = h5_file.values()[0]
    print("load data size:", values.shape[0])
    return values


def data_splitting(feature):
    '''
    Split data into two categories: go and cut-in.
    '''

    # Don't consider those anomaly data
    idx_normal = (feature[:, -3] != -10)
    go_idx = (feature[:, -3] % 2 == 0)
    cutin_idx = (feature[:, -3] % 2 == 1)

    go_idx = np.logical_and(go_idx, idx_normal)
    cutin_idx = np.logical_and(cutin_idx, idx_normal)

    feature = np.asarray(feature)

    return feature[go_idx], feature[cutin_idx]


def down_sample(feature, label, drop_rate):
    '''
    feature: the input data
    label and drop_rate: one-to-one mapping of the drop-rate for each
    specific label
    '''
    fea_label = feature[:, -2]
    selected_idx = np.zeros(fea_label.shape[0], dtype=bool)
    mask_random = np.random.random(fea_label.shape[0])

    for i in range(len(label)):
        l = label[i]
        dr = drop_rate[i]

        idx_of_curr_label = (fea_label == l)
        selected_idx_of_curr_label = np.logical_and(idx_of_curr_label,
                                                    mask_random > dr)
        selected_idx = np.logical_or(selected_idx, selected_idx_of_curr_label)

    data_downsampled = feature[selected_idx, :]
    return data_downsampled


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Data preprocessing.')
    parser.add_argument('directory', type=str,
                        help='directory contains feature files in .h5')
    parser.add_argument('-m', '--merge_files', action='store_true',
                        help='Merge output files into one.')
    parser.add_argument('-s', '--split_category', action='store_true',
                        help='Split the output into Go and Cutin.')

    args = parser.parse_args()

    path = args.directory
    print("Loading h5 from directory: {}".format(path))

    if not args.merge_files:

        if os.path.isdir(path):

            h5_files = getListOfFiles(path)
            print("Total number of files:", len(h5_files))

            # For each file in the total list of files:
            for i, file in enumerate(h5_files):
                print("Process File", i, ":", file)
                feature = load_hdf5(file)
                if np.any(np.isinf(feature)):
                    print("inf data found")

                if args.split_category:
                    # Split data into two categories:
                    fea_go, fea_cutin = data_splitting(feature)

                    # Balance data by down-sampling oversized bins:
                    #fea_go = down_sample(fea_go, [0, 1, 4], [0.0, 0.95, 0.83])
                    #fea_cutin = down_sample(fea_cutin, [-1, 2, 3], [0.985 ,0.0, 0.0])

                    go_path = path + 'go/' + \
                        file.split('/')[-2] + '-' + file.split('/')[-1]
                    h5_file = h5py.File(go_path, 'w')
                    h5_file.create_dataset('data', data=fea_go)
                    h5_file.close()

                    cutin_path = path + 'cutin/' + \
                        file.split('/')[-2] + '-' + file.split('/')[-1]
                    h5_file = h5py.File(cutin_path, 'w')
                    h5_file.create_dataset('data', data=fea_cutin)
                    h5_file.close()

                else:
                    print(None)
                    # TODO: implement those non-splitting category
        else:
            print("Fail to find", path)
            os._exit(-1)

    else:

        if os.path.isdir(path):
            features_go = None
            features_cutin = None
            features = None
            labels = None

            h5_files = getListOfFiles(path)
            print("Total number of files:", len(h5_files))

            # For each file in the total list of files:
            for i, file in enumerate(h5_files):
                print("Process File", i, ":", file)
                feature = load_hdf5(file)
                if np.any(np.isinf(feature)):
                    print("inf data found")

                if args.split_category:
                    # Split data into two categories:
                    fea_go, fea_cutin = data_splitting(feature)

                    # Balance data by down-sampling oversized bins:
                    #fea_go = down_sample(fea_go, [0, 1, 4], [0.0, 0.95, 0.83])
                    #fea_cutin = down_sample(fea_cutin, [-1, 2, 3], [0.985 ,0.0, 0.0])

                    features_go = np.concatenate((features_go, fea_go), axis=0) if features_go is not None \
                        else fea_go
                    features_cutin = np.concatenate((features_cutin, fea_cutin), axis=0) if features_cutin is not None \
                        else fea_cutin
        else:
            print("Fail to find", path)
            os._exit(-1)

        if args.split_category:
            date = datetime.datetime.now().strftime('%Y-%m-%d')
            sample_file = path + 'merged_go' + date + '.h5'
            print("Save samples file to:", sample_file)
            h5_file = h5py.File(sample_file, 'w')
            h5_file.create_dataset('data', data=features_go)
            h5_file.close()

            sample_file = path + 'merged_cutin' + date + '.h5'
            print("Save samples file to:", sample_file)
            h5_file = h5py.File(sample_file, 'w')
            h5_file.create_dataset('data', data=features_cutin)
            h5_file.close()
