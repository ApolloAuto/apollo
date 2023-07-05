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
import glob
import logging
import os
import sys

from modules.tools.prediction.data_pipelines.common.online_to_offline import LabelGenerator


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Generate labels')
    parser.add_argument('input', type=str, help='input file')
    args = parser.parse_args()

    label_gen = LabelGenerator()

    print("Create Label {}".format(args.input))
    if os.path.isfile(args.input):
        label_gen.LoadFeaturePBAndSaveLabelFiles(args.input)
        label_gen.LabelJunctionExit()
    else:
        print("{} is not a valid file".format(args.input))
