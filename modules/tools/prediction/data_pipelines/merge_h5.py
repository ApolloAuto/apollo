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

import argparse
import datetime
import os

import numpy as np
import h5py


def getListOfFiles(dirName):
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
    values = list(h5_file.values())[0]
    print("load data size:", values.shape[0])
    return values


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='generate training samples\
            from a specified directory')
    parser.add_argument('directory', type=str,
                        help='directory contains feature files in .h5')
    parser.add_argument('-n', '--npy', action='store_true',
                        help='if is .npy rather than .h5, use this.')
    args = parser.parse_args()
    path = args.directory

    if not args.npy:
        print("load h5 from directory: {}".format(path))
        if os.path.isdir(path):
            features = None
            labels = None

            h5_files = getListOfFiles(path)
            print("Total number of files:", len(h5_files))
            for i, h5_file in enumerate(h5_files):
                print("Process File", i, ":", h5_file)
                feature = load_hdf5(h5_file)
                if np.any(np.isinf(feature)):
                    print("inf data found")
                features = np.concatenate((features, feature), axis=0) if features is not None \
                    else feature
        else:
            print("Fail to find", path)
            os._exit(-1)

        date = datetime.datetime.now().strftime('%Y-%m-%d')
        sample_file = path + '/merged' + date + '.h5'
        print("Save samples file to:", sample_file)
        h5_file = h5py.File(sample_file, 'w')
        h5_file.create_dataset('data', data=features)
        h5_file.close()
    else:
        print("load npy from directory: {}".format(path))
        if os.path.isdir(path):
            features_go = None
            features_cutin = None
            npy_files = getListOfFiles(path)
            print("Total number of files:", len(npy_files))
            for i, npy_file in enumerate(npy_files):
                print("Process File", i, ":", npy_file)
                temp_features = np.load(npy_file)
                feature_go = np.zeros((temp_features.shape[0], 157))
                feature_cutin = np.zeros((temp_features.shape[0], 157))
                count_go = 0
                count_cutin = 0
                for j in range(temp_features.shape[0]):
                    fea = np.asarray(temp_features[j])
                    if fea.shape[0] != 157:
                        continue
                    if fea[-1] < -1 or fea[-1] > 4:
                        continue
                    fea = fea.reshape((1, 157))
                    if fea[0, -1] % 2 == 0:
                        feature_go[count_go] = fea
                        count_go += 1
                    else:
                        feature_cutin[count_cutin] = fea
                        count_cutin += 1

                feature_go = feature_go[:count_go]
                feature_cutin = feature_cutin[:count_cutin]
                features_go = np.concatenate((features_go, feature_go), axis=0) if features_go is not None \
                    else feature_go
                features_cutin = np.concatenate((features_cutin, feature_cutin), axis=0) if features_cutin is not None \
                    else feature_cutin
        else:
            print("Fail to find", path)
            os._exit(-1)

        print(features_go.shape)
        print(features_cutin.shape)
        date = datetime.datetime.now().strftime('%Y-%m-%d')
        sample_file_go = path + '/merged_go_' + date + '.h5'
        sample_file_cutin = path + '/merged_cutin_' + date + '.h5'
        h5_file_go = h5py.File(sample_file_go, 'w')
        h5_file_go.create_dataset('data', data=features_go)
        h5_file_go.close()
        h5_file_cutin = h5py.File(sample_file_cutin, 'w')
        h5_file_cutin.create_dataset('data', data=features_cutin)
        h5_file_cutin.close()
