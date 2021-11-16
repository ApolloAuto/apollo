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

import argparse
import os
import re

from modules.tools.prediction.data_pipelines.data_preprocessing.features_labels_utils import CombineFeaturesAndLabels


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Merge all label_dicts in each terminal folder.')
    parser.add_argument('features_dirpath', type=str,
                        help='Path of terminal folder for data_for_learn.')
    parser.add_argument('labels_dirpath', type=str,
                        help='Path of terminal folder for labels')
    args = parser.parse_args()

    list_of_files = os.listdir(args.features_dirpath)
    for file in list_of_files:
        full_file_path = os.path.join(args.features_dirpath, file)
        if file.split('.')[-1] == 'bin' and \
           file.split('.')[0] == 'datalearn':
            label_path = args.labels_dirpath
            CombineFeaturesAndLabels(full_file_path, label_path +
                                     '/junction_label.npy', 'junction_label')
