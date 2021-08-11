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

import sys

import matplotlib.pyplot as plt
import numpy as np
import tkinter.filedialog

from modules.tools.vehicle_calibration.process import get_start_index
from modules.tools.vehicle_calibration.process import preprocess
from modules.tools.vehicle_calibration.process import process


class Plotter(object):
    """
    Plot the speed info
    """

    def __init__(self):
        """
        Init the speed info
        """
        np.set_printoptions(precision=3)
        self.file = open('temp_result.csv', 'a')

    def process_data(self, filename):
        """
        Load the file and preprocess th data
        """
        self.data = preprocess(filename)

        self.tablecmd, self.tablespeed, self.tableacc, self.speedsection, self.accsection, self.timesection = process(
            self.data)

    def plot_result(self):
        """
        Plot the desired data
        """
        fig, axarr = plt.subplots(2, 1, sharex=True)
        plt.tight_layout()
        fig.subplots_adjust(hspace=0)

        axarr[0].plot(
            self.data['time'], self.data['ctlbrake'], label='Brake CMD')
        axarr[0].plot(
            self.data['time'],
            self.data['brake_percentage'],
            label='Brake Output')
        axarr[0].plot(
            self.data['time'], self.data['ctlthrottle'], label='Throttle CMD')
        axarr[0].plot(
            self.data['time'],
            self.data['throttle_percentage'],
            label='Throttle Output')
        axarr[0].plot(
            self.data['time'],
            self.data['engine_rpm'] / 100,
            label='Engine RPM')
        axarr[0].legend(fontsize='medium')
        axarr[0].grid(True)
        axarr[0].set_title('Command')

        axarr[1].plot(
            self.data['time'],
            self.data['vehicle_speed'],
            label='Vehicle Speed')

        for i in range(len(self.timesection)):
            axarr[1].plot(
                self.timesection[i],
                self.speedsection[i],
                label='Speed Segment')
            axarr[1].plot(
                self.timesection[i], self.accsection[i], label='IMU Segment')

        axarr[1].legend(fontsize='medium')
        axarr[1].grid(True)
        axarr[1].set_title('Speed')

        mng = plt.get_current_fig_manager()
        mng.full_screen_toggle()

        # plt.tight_layout(pad=0.20)
        fig.canvas.mpl_connect('key_press_event', self.press)
        plt.show()

    def press(self, event):
        """
        Keyboard events during plotting
        """
        if event.key == 'q' or event.key == 'Q':
            self.file.close()
            plt.close()

        if event.key == 'w' or event.key == 'W':
            for i in range(len(self.tablecmd)):
                for j in range(len(self.tablespeed[i])):
                    self.file.write("%s, %s, %s\n" %
                                    (self.tablecmd[i], self.tablespeed[i][j],
                                     self.tableacc[i][j]))
            print("Finished writing results")


def main():
    """
    demo
    """
    if len(sys.argv) == 2:
        # Get the latest file
        file_path = sys.argv[1]
    else:
        file_path = tkinter.filedialog.askopenfilename(
            initialdir="/home/caros/.ros",
            filetypes=(("csv files", ".csv"), ("all files", "*.*")))
    print('File path: %s' % file_path)
    plotter = Plotter()
    plotter.process_data(file_path)
    print('Finished reading the file.')
    plotter.plot_result()


if __name__ == '__main__':
    main()
