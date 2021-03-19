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


class Path:
    def __init__(self, path_files):
        self.path_files = path_files

    def draw(self, ax):
        xs = []
        ys = []
        for path_file in self.path_files:
            with open(path_file, 'r') as f:
                lines = f.readlines()
                for line in lines:
                    items = line.split(',')
                    xs.append(float(items[1]))
                    ys.append(float(items[2]))
        ax.plot(xs, ys, ls='--', c='k', alpha=0.5)
