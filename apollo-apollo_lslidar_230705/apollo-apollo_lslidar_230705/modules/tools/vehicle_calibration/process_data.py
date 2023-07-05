#!/usr/bin/env python3

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
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
"""
This module provide function to plot the speed control info from log csv file
"""

import math
import sys

import numpy as np
import tkinter.filedialog

from modules.tools.vehicle_calibration.process import get_start_index
from modules.tools.vehicle_calibration.process import preprocess
from modules.tools.vehicle_calibration.process import process


class Plotter(object):
    """
    plot the speed info
    """

    def __init__(self, filename):
        """
        init the speed info
        """

        np.set_printoptions(precision=3)
        self.file = open('result.csv', 'a')
        self.file_one = open(filename + ".result", 'w')

    def process_data(self, filename):
        """
        load the file and preprocess th data
        """

        self.data = preprocess(filename)

        self.tablecmd, self.tablespeed, self.tableacc, self.speedsection, self.accsection, self.timesection = process(
            self.data)

    def save_data(self):
        """
        save_data
        """
        for i in range(len(self.tablecmd)):
            for j in range(len(self.tablespeed[i])):
                self.file.write("%s, %s, %s\n" %
                                (self.tablecmd[i], self.tablespeed[i][j],
                                 self.tableacc[i][j]))
                self.file_one.write("%s, %s, %s\n" %
                                    (self.tablecmd[i], self.tablespeed[i][j],
                                     self.tableacc[i][j]))


def main():
    """
    demo
    """
    if len(sys.argv) == 2:
        # get the latest file
        file_path = sys.argv[1]
    else:
        file_path = tkinter.filedialog.askopenfilename(
            initialdir="/home/caros/.ros",
            filetypes=(("csv files", ".csv"), ("all files", "*.*")))
    plotter = Plotter(file_path)
    plotter.process_data(file_path)
    plotter.save_data()
    print('save result to:', file_path + ".result")


if __name__ == '__main__':
    main()
