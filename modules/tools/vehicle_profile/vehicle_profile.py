#!/usr/bin/env python
# -*- coding:UTF-8 -*-
###############################################################################
# Copyright 2023 The Apollo Authors. All Rights Reserved.
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
Control Planning Analyzer
"""
import argparse
import os
import math
import sys
import threading
import time
import matplotlib
import matplotlib.pyplot as plt
import numpy
import fnmatch
# import xlrd
import xlsxwriter

from matplotlib import patches
from matplotlib import lines

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3.record import RecordReader

from modules.common_msgs.chassis_msgs import chassis_pb2
from modules.common_msgs.control_msgs import control_cmd_pb2
from modules.common_msgs.localization_msgs import localization_pb2
from modules.common_msgs.chassis_msgs import chassis_pb2
from modules.common_msgs.planning_msgs import planning_pb2
from modules.common_msgs.control_msgs import control_cmd_pb2

import importlib

importlib.reload(sys)


class ControlInfo(object):
    """
    ControlInfo Class
    """

    def __init__(self, axarr, t=False):

        self.controltime = []
        self.canbustime = []
        # front and back information
        self.station_reference = []
        self.current_station = []
        self.station_error = []
        # speed information
        self.canbus_speed = []
        self.speed_reference = []
        self.current_speed = []
        self.speed_error = []
        # left and right information
        self.lateral_error = []
        # heading information
        self.ref_heading = []
        self.heading = []
        self.heading_error = []
        self.driving_mode = 0
        self.mode_time = []
        self.target_time = []
        self.axarr = axarr
        self.print_traj = False
        self.planningavailable = False
        self.lock = threading.Lock()
        self.carx = []
        self.cary = []
        self.cartime = []
        self.pathx = []
        self.pathy = []
        self.planningx = []
        self.planningy = []
        self.i = 0
        self.score_num = 0

    def callback_planning(self, entity):
        """
        New Planning Trajectory
        """
        basetime = entity.header.timestamp_sec
        self.pathx = [p.path_point.x for p in entity.trajectory_point]
        self.pathy = [p.path_point.y for p in entity.trajectory_point]
        if self.i < 40:
            self.planningx = self.pathx
            self.planningy = self.pathy

        self.i += 1
        numpoints = len(entity.trajectory_point)
        if numpoints == 0:
            self.planningavailable = False
        else:
            self.planningavailable = True

    def callback_control(self, entity):
        """
        New Control Command
        """
        self.controltime.append(entity.header.timestamp_sec)
        #evaluation of front and back distance
        self.station_reference.append(
            entity.debug.simple_lon_debug.station_reference)
        self.current_station.append(
            entity.debug.simple_lon_debug.current_station)
        self.station_error.append(entity.debug.simple_lon_debug.station_error)

        #evaluation of speed
        self.speed_reference.append(
            entity.debug.simple_lon_debug.speed_reference)
        #print"speed_reference is \n", self.speed_reference
        self.current_speed.append(
            entity.debug.simple_lon_debug.speed_reference -
            entity.debug.simple_lon_debug.speed_error)
        #print"current_speed is \n", self.current_speed
        self.speed_error.append(entity.debug.simple_lon_debug.speed_error)
        #evaluation of left and right
        self.lateral_error.append(entity.debug.simple_lat_debug.lateral_error)
        #evaluation of heading error
        self.ref_heading.append(entity.debug.simple_lat_debug.ref_heading)
        self.heading.append(entity.debug.simple_lat_debug.heading)
        self.heading_error.append(entity.debug.simple_lat_debug.heading_error)
        with self.lock:
            if self.planningavailable:
                self.target_time.append(entity.header.timestamp_sec)

    def callback_canbus(self, entity):
        """
        New Canbus
        """
        self.canbus_speed.append(entity.speed_mps)
        self.canbustime.append(entity.header.timestamp_sec)

        if entity.driving_mode == chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE:
            if self.driving_mode == 0:
                self.mode_time.append(entity.header.timestamp_sec)
                self.driving_mode = 1
        elif self.driving_mode == 1:
            self.mode_time.append(entity.header.timestamp_sec)
            self.driving_mode = 0

    def callback_localization(self, entity):
        """
        New localization pose
        """
        self.carx.append(entity.pose.position.x)
        self.cary.append(entity.pose.position.y)
        self.cartime.append(entity.header.timestamp_sec)

    def control_planning_dif(self, file, t=False):
        """
        Showing Longitudinal
        """
        fig, ax = plt.subplots(2, 2)
        ax[0, 0].get_shared_x_axes().join(ax[0, 0], ax[0, 0])
        ax[0, 0].get_shared_x_axes().join(ax[0, 0], ax[0, 0])
        ax[0, 0].get_shared_x_axes().join(ax[0, 0], ax[0, 1])
        ax[0, 0].get_shared_x_axes().join(ax[0, 0], ax[1, 0])
        ax[0, 0].get_shared_x_axes().join(ax[0, 0], ax[1, 1])
        # del_head_error = 100
        # del_head_error = self.calculate_head_error(self.current_speed)
        if self.current_speed:
            del_head_error = self.calculate_head_error(self.current_speed)
        else:
            del_head_error = self.calculate_head_error(self.canbus_speed)
        # del_head_error = 100
        # print('del_head_error is %',  del_head_error)
        if self.current_station:
            del_end_error = self.calculate_end_error(self.current_station)
        self.print_traj = t

        #station_error
        self.station_reference = self.station_reference[
            del_head_error:del_end_error]
        # for sta in self.station_reference:
        #     print(sta)
        self.current_station = self.current_station[
            del_head_error:del_end_error]
        max_station = max(self.current_station)
        # print "Vehicel max drive distance is: %.0fm" % int(max_station)
        self.station_error = self.station_error[del_head_error:del_end_error]
        self.controltime = self.controltime[del_head_error:del_end_error]

        if not self.print_traj:
            ax[0, 0].plot(self.controltime,
                          self.station_reference,
                          linewidth=1.0,
                          label='station_reference')
            #self.controltime, self.station_reference)
            ax[0, 0].plot(self.controltime,
                          self.current_station,
                          linewidth=1.0,
                          label='current_station')
            #self.controltime, self.curre302,nt_station)
            ax[0, 0].plot(self.controltime,
                          self.station_error,
                          linewidth=1.0,
                          label='station_error')
            #self.controltime, self.station_error)
            # legend=ax[0, 0].legend(fontsize='small', loc='upper left')
            legend = ax[0, 0].legend(fontsize='small')
            frame = legend.get_frame()
            frame.set_alpha(1)
            frame.set_facecolor('none')
            ax[0, 0].grid(True)
            ax[0, 0].set_title('station information')
            ax[0, 0].set_xlabel('Time')
        # python3 map return object not list
        abs_station_error = list(map(abs, self.station_error))
        max_station_error = max(abs_station_error)
        min_station_error = min(abs_station_error)
        average_station_error = self.calculate_average(abs_station_error)

        #lateral_error
        self.lateral_error = self.lateral_error[del_head_error:del_end_error]
        if not self.print_traj:
            ax[0, 1].plot(self.controltime,
                          self.lateral_error,
                          linewidth=1.0,
                          label='lateral_error')
            #self.controltime, self.lateral_error)
            # legend=ax[0, 1].legend(fontsize='small',loc='upper left')
            legend = ax[0, 1].legend(fontsize='small')
            frame = legend.get_frame()
            frame.set_alpha(1)
            frame.set_facecolor('none')
            ax[0, 1].grid(True)
            ax[0, 1].set_title('lateral Information')
            ax[0, 1].set_xlabel('Time')
        abs_lateral_error = list(map(abs, self.lateral_error))
        max_lateral_error = max(abs_lateral_error)
        min_lateral_error = min(abs_lateral_error)
        average_lateral_error = self.calculate_average(abs_lateral_error)

        # speed_error
        self.speed_reference = self.speed_reference[
            del_head_error:del_end_error]
        self.current_speed = self.current_speed[del_head_error:del_end_error]
        # self.speed_error = self.speed_error[del_head_error:del_end_error]
        self.canbus_speed = self.canbus_speed[del_head_error:del_end_error]
        self.canbustime = self.canbustime[del_head_error:del_end_error]
        max_speed = max(self.current_speed)
        max_chassis_speed = max(self.canbus_speed)
        if not self.print_traj:
            print("Vehicel max speed is: %.3fm/s" % max_speed)
            # print("Vehicel max chassis_speed is: %.3fm/s" % max_chassis_speed)
        self.speed_error = self.speed_error[del_head_error:del_end_error]
        if not self.print_traj:
            ax[1, 0].plot(self.canbustime,
                          self.canbus_speed,
                          linewidth=1.0,
                          label='canbus_speed')
            ax[1, 0].plot(self.controltime,
                          self.speed_reference,
                          linewidth=1.0,
                          label='speed_reference')
            #self.controltime, self.speed_reference)
            ax[1, 0].plot(self.controltime,
                          self.current_speed,
                          linewidth=1.0,
                          label='current_speed')
            #self.controltime, self.current_speed)
            ax[1, 0].plot(self.controltime,
                          self.speed_error,
                          linewidth=1.0,
                          label='speed_error')
            #self.controltime, self.speed_error)
            legend = ax[1, 0].legend(fontsize='small', loc='upper left')
            frame = legend.get_frame()
            frame.set_alpha(1)
            frame.set_facecolor('none')
            ax[1, 0].grid(True)
            ax[1, 0].set_title('Speed information')
            ax[1, 0].set_xlabel('Time')
        abs_speed_error = list(map(abs, self.speed_error))
        max_speed_error = max(abs_speed_error)
        min_speed_error = min(abs_speed_error)
        average_speed_error = self.calculate_average(abs_speed_error)

        #heading_error
        self.ref_heading = self.ref_heading[del_head_error:del_end_error]
        self.heading = self.heading[del_head_error:del_end_error]
        vehicel_behavior = self.get_vehicle_behavior(self.heading)
        # print "Vehicel host behavior is: %s" % vehicel_behavior
        self.heading_error = self.heading_error[del_head_error:del_end_error]
        if not self.print_traj:
            ax[1, 1].plot(self.controltime,
                          self.ref_heading,
                          linewidth=1.0,
                          label='ref_heading')
            #self.controltime, self.ref_heading)
            ax[1, 1].plot(self.controltime,
                          self.heading,
                          linewidth=1.0,
                          label='heading')
            #self.controltime, self.heading)
            ax[1, 1].plot(self.controltime,
                          self.heading_error,
                          linewidth=1.0,
                          label='heading_error')
            #self.controltime, self.heading_error)
            legend = ax[1, 1].legend(
                fontsize='small'
            )  #, mode='expend', shadow=True, bbox_to_anchor=(1.05,1), loc = 'upper left', borderaxespad=0.
            frame = legend.get_frame()
            frame.set_alpha(1)
            frame.set_facecolor('none')
            ax[1, 1].grid(True)
            ax[1, 1].set_title('Heading information')
            ax[1, 1].set_xlabel('Time')
        abs_heading_error = list(map(abs, self.heading_error))
        max_heading_error = max(abs_heading_error)
        min_heading_error = min(abs_heading_error)
        average_heading_error = self.calculate_average(abs_heading_error)

        # Plot Trajectory
        # self.carx = self.carx[del_head_error:del_end_error]
        # self.cary = self.cary[del_head_error:del_end_error]
        # self.pathx = self.pathx[del_head_error:del_end_error]
        # self.pathy = self.pathy[del_head_error:del_end_error]
        # self.planningx = self.planningx[del_head_error:del_end_error]
        # self.planningy = self.planningy[del_head_error:del_end_error]

        if self.print_traj:
            # print('carx list length is %',  len(self.carx))
            # print('pathx list length is %',  len(self.pathx))
            fig, ax = plt.subplots(1, 1)
            ax.plot(self.planningx,
                    self.planningy,
                    linewidth=2.0,
                    linestyle='--',
                    label='planning path')
            ax.plot(self.carx, self.cary, linewidth=1.0, label='pose')
            ax.plot(self.pathx,
                    self.pathy,
                    linewidth=1.0,
                    label='planning path remain')
            legend = ax.legend(fontsize='small')
            frame = legend.get_frame()
            frame.set_alpha(1)
            frame.set_facecolor('none')
            ax.grid(True)
            ax.set_title('Trajectory')
            ax.set_xlabel('x')
            ax.set_ylabel('y')

        #write result to excel and calculate grade
        #book_format = book.add_format({'font_color': 'black', 'align': 'center'})
        # 设置单元格居中，自动换行
        book_format = book.add_format({'align': 'center', 'text_wrap': True})
        sheet.write(file_num, 0, file)
        sheet.write(file_num, 1, format(max_station_error, '.3f'), book_format)
        sheet.write(file_num, 2, format(min_station_error, '.3f'), book_format)
        sheet.write(file_num, 3, format(average_station_error, '.3f'),
                    book_format)
        station_error_grade = self.get_station_error_grade(max_station_error)
        station_error_stype = self.get_grade_style(int(station_error_grade))
        station_error_stype.set_bold()
        sheet.write(file_num, 4, format(station_error_grade, '.2f'),
                    station_error_stype)

        sheet.write(file_num, 5, format(max_lateral_error, '.3f'), book_format)
        sheet.write(file_num, 6, format(min_lateral_error, '.3f'), book_format)
        sheet.write(file_num, 7, format(average_lateral_error, '.3f'),
                    book_format)
        lateral_error_grade = self.get_lateral_error_grade(max_lateral_error)
        lateral_error_stype = self.get_grade_style(int(lateral_error_grade))
        lateral_error_stype.set_bold()
        sheet.write(file_num, 8, format(lateral_error_grade, '.2f'),
                    lateral_error_stype)

        sheet.write(file_num, 9, format(max_speed_error, '.3f'), book_format)
        sheet.write(file_num, 10, format(min_speed_error, '.3f'), book_format)
        sheet.write(file_num, 11, format(average_speed_error, '.3f'),
                    book_format)
        speed_error_grade = self.get_speed_error_grade(max_speed_error)
        speed_error_stype = self.get_grade_style(int(speed_error_grade))
        speed_error_stype.set_bold()
        sheet.write(file_num, 12, format(speed_error_grade, '.2f'),
                    speed_error_stype)

        sheet.write(file_num, 13, format(max_heading_error, '.3f'), book_format)
        sheet.write(file_num, 14, format(min_heading_error, '.3f'), book_format)
        sheet.write(file_num, 15, format(average_heading_error, '.3f'),
                    book_format)
        heading_error_grade = self.get_heading_error_grade(max_heading_error)
        heading_error_stype = self.get_grade_style(int(heading_error_grade))
        heading_error_stype.set_bold()
        sheet.write(file_num, 16, format(heading_error_grade, '.2f'),
                    heading_error_stype)

        average_score = format((station_error_grade + lateral_error_grade +
                                speed_error_grade + heading_error_grade) / 4.0,
                               '.2f')
        sheet.write(file_num, 17, average_score, book_format)

        sheet.write(file_num, 18, vehicel_behavior, book_format)
        sheet.write(file_num, 19, format(max_speed, '.3f'), book_format)
        sheet.write(file_num, 20, int(max_station), book_format)

        # debug for heading/localization/path
        # sheet.write_column('V2', self.pathx)
        # sheet.write_column('W2', self.pathy)
        # sheet.write_column('X2', self.cartime)
        # debug for speed
        # sheet.write_column('Y2', self.speed_reference)
        # sheet.write_column('Z2', self.current_station)
        # sheet.write_column('AA2', self.speed_error)

        if args.path and not self.print_traj:
            average_score = format(
                (station_error_grade + lateral_error_grade + speed_error_grade +
                 heading_error_grade) / 4.0, '.2f')
            # print(type(average_score))
            # global score_num
            # score_num = 0
            self.score_num = float(self.score_num) + float(average_score)
            # score_num = score_num + average_score
            #book_format = book.add_format({'align':'center','text_wrap': True})
            book_format17 = book.add_format({
                'font_color': 'black',
                'align': 'center'
            })
            book_format17.set_bold()
            case_sheet.write(record_num, 0, file)
            case_sheet.write(record_num, 1, vehicel_behavior, book_format)
            case_sheet.write(record_num, 2, format(max_speed, '.3f'),
                             book_format)
            case_sheet.write(record_num, 3, int(max_station), book_format)
            case_sheet.write(record_num, 5, format(max_station_error, '.3f'),
                             book_format)
            case_sheet.write(record_num, 6, format(average_station_error,
                                                   '.3f'), book_format)
            case_sheet.write(record_num, 7, format(station_error_grade, '.2f'),
                             station_error_stype)
            case_sheet.write(record_num, 8, format(max_lateral_error, '.3f'),
                             book_format)
            case_sheet.write(record_num, 9, format(average_lateral_error,
                                                   '.3f'), book_format)
            case_sheet.write(record_num, 10, format(lateral_error_grade, '.2f'),
                             lateral_error_stype)
            case_sheet.write(record_num, 11, format(max_speed_error, '.3f'),
                             book_format)
            case_sheet.write(record_num, 12, format(average_speed_error, '.3f'),
                             book_format)
            case_sheet.write(record_num, 13, format(speed_error_grade, '.2f'),
                             speed_error_stype)
            case_sheet.write(record_num, 14, format(max_heading_error, '.3f'),
                             book_format)
            case_sheet.write(record_num, 15, format(average_heading_error,
                                                    '.3f'), book_format)
            case_sheet.write(record_num, 16, format(heading_error_grade, '.2f'),
                             heading_error_stype)
            case_sheet.write(record_num, 17, average_score, book_format)

        if not self.print_traj:
            if len(self.mode_time) % 2 == 1:
                self.mode_time.append(self.controltime[-1])
            for i in range(0, len(self.mode_time), 2):
                ax[0, 0].axvspan(self.mode_time[i],
                                 self.mode_time[i + 1],
                                 fc='0.1',
                                 alpha=0.1)
                ax[1, 0].axvspan(self.mode_time[i],
                                 self.mode_time[i + 1],
                                 fc='0.1',
                                 alpha=0.1)
                ax[1, 1].axvspan(self.mode_time[i],
                                 self.mode_time[i + 1],
                                 fc='0.1',
                                 alpha=0.1)

    def plot_chassis_speed(self):

        fig, ax = plt.subplots(1, 1)
        ax.plot(self.canbustime,
                self.canbus_speed,
                linewidth=1.0,
                label='canbus_speed')
        max_chassis_speed = max(self.canbus_speed)
        print("Vehicel max chassis_speed is: %.3fm/s" % max_chassis_speed)

    def press(self, event):
        """
        Keyboard events during plotting
        """
        if event.key == 'q' or event.key == 'Q':
            plt.close('all')
        if event.key == 'd' or event.key == 'D':
            self.control_planning_dif(file)

    def calculate_average(self, array):
        sum = 0
        num = len(array)
        for i in array:
            sum = sum + abs(i)
        return sum / num

    def calculate_head_error(self, list):
        # print("list head data length is ", len(list))
        for i in range(3, len(list)):
            if list[i + 1] - list[i] >= 0:
                # print("head_i=", i)
                head = i
                break
        return head

    def calculate_end_error(self, list):
        for i in range(len(list), 0, -1):
            if list[i - 1] != 0:
                # print("end_i=", i)
                end = i - 2
                break
        return end

    def get_vehicle_behavior(self, heading, del_head_error=1):
        k = 57.3248
        for i in range(len(heading), 0, -1):
            if heading[i - 1] != 0:
                # print("heading_end_i =", i)
                end = i - 2
                break
        # begin = del_head_error + 10
        begin = del_head_error
        # print("heading_begin_i =", begin)
        heading_begin = self.heading[begin]
        # print("heading_begin =", heading_begin)
        heading_end = self.heading[end]
        # print("heading_end =", heading_end)
        delta = (heading_end - heading_begin) * k
        # print("delta is", delta)
        behavior = ''
        if -20 < delta < 20:
            #behavior = 'go straight'
            behavior = '直行'
        elif 45 < delta < 135 or -315 < delta < -225:
            #behavior = 'turn left'
            behavior = '左转'
        elif -135 < delta < -45 or 225 < delta < 315:
            #behavior = 'turn right'
            behavior = '右转'
        return behavior

    def get_station_error_grade(self, value):
        grade = 0
        if value >= 10:
            grade = 1
        elif 6 <= value < 10:
            grade = 3 - (value - 6) / 4
        elif 3 <= value < 6:
            grade = 4 - (value - 3) / 3
        elif 1 <= value < 3:
            grade = 5 - (value - 1) / 2
        elif 0 <= value < 1:
            grade = 5
        return grade

    def get_lateral_error_grade(self, value):
        grade = 0
        if value >= 2:
            grade = 1
        elif 1 <= value < 2:
            grade = 3 - (value - 1) / 1
        elif 0.5 <= value < 1:
            grade = 4 - (value - 0.5) / 0.5
        elif 0.2 <= value < 0.5:
            grade = 5 - (value - 0.2) / 0.3
        elif 0 <= value < 0.2:
            grade = 5
        return grade

    def get_speed_error_grade(self, value):
        grade = 0
        if value >= 5:
            grade = 1
        elif 3 <= value < 5:
            grade = 3 - (value - 3) / 2
        elif 1.5 <= value < 3:
            grade = 4 - (value - 1.5) / 1.5
        elif 0.5 <= value < 1.5:
            grade = 5 - (value - 0.5) / 1
        elif 0 <= value < 0.5:
            grade = 5
        return grade

    def get_heading_error_grade(self, value):
        grade = 0
        if value >= 0.6:
            grade = 1
        elif 0.4 <= value < 0.6:
            grade = 3 - (value - 0.4) / 0.2
        elif 0.2 <= value < 0.4:
            grade = 4 - (value - 0.2) / 0.2
        elif 0.1 <= value < 0.2:
            grade = 5 - (value - 0.1) / 0.1
        elif 0 <= value < 0.1:
            grade = 5
        return grade

    def get_grade_style(self, value):
        style = ""
        style_dict = {
            1: book.add_format({
                'align': 'center',
                'font_color': 'red'
            }),
            2: book.add_format({
                'align': 'center',
                'font_color': 'brown'
            }),
            3: book.add_format({
                'align': 'center',
                'font_color': 'orange'
            }),
            4: book.add_format({
                'align': 'center',
                'font_color': 'blue'
            }),
            5: book.add_format({
                'align': 'center',
                'font_color': 'dark_green'
            })
        }
        if value == 5:
            style = style_dict[5]
        elif value == 4:
            style = style_dict[4]
        elif value == 3:
            style = style_dict[3]
        elif value == 2:
            style = style_dict[2]
        elif value == 1:
            style = style_dict[1]
        return style

    def read_bag(self, bag_file):
        file_path = bag_file
        bag = RecordReader(file_path)
        print("Begin reading the file: ", file_path)
        for msg in bag.read_messages():
            #print msg.timestamp,msg.topic
            if msg.topic == "/apollo/control":
                control_cmd = control_cmd_pb2.ControlCommand()
                control_cmd.ParseFromString(msg.message)
                controlinfo.callback_control(control_cmd)
            elif msg.topic == "/apollo/planning":
                adc_trajectory = planning_pb2.ADCTrajectory()
                #planning_pb = proto_utils.get_pb_from_text_file(planning_pb2, adc_trajectory)
                adc_trajectory.ParseFromString(msg.message)
                controlinfo.callback_planning(adc_trajectory)
            elif msg.topic == "/apollo/canbus/chassis":
                chassis = chassis_pb2.Chassis()
                chassis.ParseFromString(msg.message)
                controlinfo.callback_canbus(chassis)
            elif msg.topic == "/apollo/localization/pose":
                localization = localization_pb2.LocalizationEstimate()
                localization.ParseFromString(msg.message)
                controlinfo.callback_localization(localization)
        print("Done reading the file: ", file_path)

    def add_excel_sheet(self, file):
        sheet = book.add_worksheet(file)
        # 设置单元格居中，自动换行
        book_format = book.add_format({'align': 'center', 'text_wrap': True})
        sheet.write(0, 0, 'file_name', book_format)
        sheet.write(0, 1, 'max_station_error', book_format)
        sheet.write(0, 2, 'min_station_error', book_format)
        sheet.write(0, 3, 'average_station_error', book_format)
        sheet.write(0, 4, 'station_error_grade', book_format)

        sheet.write(0, 5, 'max_lateral_error', book_format)
        sheet.write(0, 6, 'min_lateral_error', book_format)
        sheet.write(0, 7, 'average_lateral_error', book_format)
        sheet.write(0, 8, 'lateral_error_grade', book_format)

        sheet.write(0, 9, 'max_speed_error', book_format)
        sheet.write(0, 10, 'min_speed_error', book_format)
        sheet.write(0, 11, 'average_speed_error', book_format)
        sheet.write(0, 12, 'speed_error_grade', book_format)

        sheet.write(0, 13, 'max_heading_error', book_format)
        sheet.write(0, 14, 'min_heading_error', book_format)
        sheet.write(0, 15, 'average_heading_error', book_format)
        sheet.write(0, 16, 'heading_error_grade', book_format)
        sheet.write(0, 17, '平均得分', book_format)
        sheet.write(0, 18, '主车行为', book_format)
        sheet.write(0, 19, '最大车速（m/s）', book_format)
        sheet.write(0, 20, '行驶距离（m）', book_format)
        # sheet.write(0, 21, 'carx', book_format)
        # sheet.write(0, 22, 'cary', book_format)
        # sheet.write(0, 23, 'cartime', book_format)
        # sheet.write(0, 24, 'speed_reference', book_format)
        # sheet.write(0, 25, 'current_speed', book_format)
        # sheet.write(0, 26, 'speed_error', book_format)
        # 设置0行行高
        sheet.set_row(0, 45)
        sheet.set_row(1, 25)
        # 设置0列每个单元格宽度
        sheet.set_column(0, 0, 12)
        # 设置1-19列每个单元格宽度
        sheet.set_column(1, 16, 8)
        sheet.set_column(17, 19, 7)
        return sheet


def add_test_case_result(book):
    case_sheet = book.add_worksheet('测试用例及结果')
    book_format = book.add_format({'align': 'center', 'text_wrap': True})
    case_sheet.merge_range(0, 0, 0, 18, 'Apollo车辆认证平台循迹测试报告')
    case_sheet.write(1, 0, '循迹数据包名称', book_format)
    case_sheet.write(1, 1, '主车行为', book_format)
    case_sheet.write(1, 2, '最大车速（m/s）', book_format)
    case_sheet.write(1, 3, '行驶距离（m）', book_format)
    case_sheet.write(1, 4, '描述', book_format)
    case_sheet.write(1, 5, '最大纵向误差：米max_station_error', book_format)
    case_sheet.write(1, 6, '平均纵向误差：米average_station_error', book_format)
    case_sheet.write(1, 7, '纵向误差评分 station_error_grade', book_format)
    case_sheet.write(1, 8, '最大横向误差：米max_lateral_error', book_format)
    case_sheet.write(1, 9, '平均横向误差：米average_lateral_error', book_format)
    case_sheet.write(1, 10, '横向误差评分 lateral_error_grade', book_format)
    case_sheet.write(1, 11, '最大速度误差：米max_speed_error', book_format)
    case_sheet.write(1, 12, '平均速度误差：米average_speed_error', book_format)
    case_sheet.write(1, 13, '速度误差评分 speed_error_grade', book_format)
    case_sheet.write(1, 14, '最大航向误差：rad  max_heading_error', book_format)
    case_sheet.write(1, 15, '平均航向误差：rad  average_heading_error', book_format)
    case_sheet.write(1, 16, '航向误差评分 heading_error_grade', book_format)
    case_sheet.write(1, 17, '综合评定（5分制）', book_format)
    case_sheet.write(1, 18, '备注（测试时，根据实际场地和驾驶安全需要适当调整转弯速度）', book_format)
    # 设置第1行行高
    case_sheet.set_row(1, 65)
    # 设置0列每个单元格宽度
    case_sheet.set_column(0, 0, 12)
    case_sheet.set_column(1, 4, 7)
    case_sheet.set_column(5, 17, 10)
    case_sheet.set_column(18, 18, 50)
    return case_sheet


def is_record_file(path):
    """Naive check if a path is a record."""
    return path.endswith('.record') or fnmatch.fnmatch(
        path, '*.record.?????') or fnmatch.fnmatch(path, '*.record.????')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Process and analyze control and planning data. usage:\
        'python control_planning_evaluation.py --path bag_file_directory_path',\
        'python control_planning_evaluation.py --bag bag_file_path' ")
    parser.add_argument('--bag', type=str, help='use cyber_recorder')
    parser.add_argument('--path', type=str, help='path for bag files')
    args = parser.parse_args()
    cyber.init()
    node = cyber.Node("control_profile")
    fig, axarr = plt.subplots()
    controlinfo = ControlInfo(axarr)
    trajectory_print = True
    read_bag_all = True
    file_num = 0
    if args.path:
        record_num = 1
        controlinfo.score_num = 0
        dir_path = args.path
        new_path = os.path.join(dir_path, 'picture')
        print("dir_path is", dir_path)
        path_file_name = os.path.splitext(args.path)[0].split('/')[2]
        print("path_file_name", path_file_name)
        excel_name = dir_path + path_file_name + '_' + 'evaluation_result.xlsx'
        print("excel_name", excel_name)
        case_file_name = dir_path + 'case.xlsx'
        flag = os.path.exists(case_file_name)
        book = xlsxwriter.Workbook(excel_name)
        case_sheet = add_test_case_result(book)
        if read_bag_all:
            if os.path.isdir(dir_path):
                dirs = os.listdir(dir_path)
                # print("dirs is", dirs)
                dirs.sort()
                if (os.path.exists(new_path) == False):
                    os.mkdir(new_path)
                    print("new path is", new_path)
                for file in dirs:
                    # print("file", file)
                    if is_record_file(file):
                        controlinfo.read_bag(dir_path + file)

            record_num = record_num + 1
            file_num = 1
            sheet = controlinfo.add_excel_sheet(path_file_name)
            # mng = plt.get_current_fig_manager()
            controlinfo.control_planning_dif(path_file_name)
            fig.canvas.mpl_connect('key_press_event', controlinfo.press)
            # plt.show()
            picture_name = new_path + '/' + path_file_name + '_' + 'picture' + '_' + '1' + '.jpg'
            print("save picture1 to: ", picture_name)
            plt.tight_layout()
            plt.subplots_adjust(right=0.8)
            plt.savefig(picture_name, dpi=300)
            sheet.insert_image(4, 1, picture_name)
            # plt trajectory
            controlinfo.control_planning_dif(path_file_name, trajectory_print)
            fig.canvas.mpl_connect('key_press_event', controlinfo.press)
            picture_name = new_path + '/' + path_file_name + '_' + 'picture' + '_' + '2' + '.jpg'
            print("save picture2 to:", picture_name)
            plt.savefig(picture_name, dpi=300)  #不设置分辨率300，不同的结果图片分辨率不同
            # sheet.insert_image(4, 11, picture_name) #设置插入到Excel内，图片的缩放，默认是1
            sheet.insert_image(4, 11, picture_name, {
                'x_scale': 0.9,
                'y_scale': 1
            })

            record_num = record_num + 1
            case_sheet.merge_range(record_num, 0, record_num, 1, '')
            case_sheet.merge_range(record_num, 2, record_num, 16, '')
            case_sheet.write(record_num, 17, '总分：' + format(controlinfo.score_num))
            record_num = record_num + 1
            string_value = "总体主观评价"
            case_sheet.merge_range(record_num, 0, record_num, 1, string_value)
            case_sheet.write(
                record_num, 17,
                '平均：' + format(controlinfo.score_num / (record_num - 3), '.3f'))
            #excel_name1 = dir_path + '/' + excel_name
            print("save all results to: ", excel_name)

            book.close()

        else:
            if os.path.isdir(dir_path):
                dirs = os.listdir(dir_path)
                dirs.sort()
                if (os.path.exists(new_path) == False):
                    os.mkdir(new_path)
                for file in dirs:
                    if is_record_file(file):
                        record_num = record_num + 1
                        file_num = 1
                        controlinfo.read_bag(dir_path + file)
                        sheet = controlinfo.add_excel_sheet(file)
                        controlinfo.control_planning_dif(file)
                        fig.canvas.mpl_connect('key_press_event',
                                               controlinfo.press)
                        picture_name = new_path + '/' + file + '_' + '1' + '.jpg'
                        print("save picture1 to: ", picture_name)
                        plt.tight_layout()
                        plt.subplots_adjust(right=0.8)
                        plt.savefig(picture_name, dpi=300)
                        sheet.insert_image(4, 1, picture_name)
                        controlinfo = ControlInfo(axarr)
                        controlinfo.read_bag(dir_path + file)
                        controlinfo.control_planning_dif(file, trajectory_print)
                        fig.canvas.mpl_connect('key_press_event',
                                               controlinfo.press)
                        picture_name = new_path + '/' + file + '_' + '2' + '.jpg'
                        print("save picture2 to:", picture_name)
                        plt.savefig(picture_name,
                                    dpi=300)  #不设置分辨率300，不同的结果图片分辨率不同
                        sheet.insert_image(4, 11, picture_name, {
                            'x_scale': 0.9,
                            'y_scale': 1
                        })
            record_num = record_num + 1
            case_sheet.merge_range(record_num, 0, record_num, 1, '')
            case_sheet.merge_range(record_num, 2, record_num, 16, '')
            case_sheet.write(record_num, 17, '总分：' + format(controlinfo.score_num))
            record_num = record_num + 1
            string_value = "总体主观评价"
            case_sheet.merge_range(record_num, 0, record_num, 1, string_value)
            case_sheet.write(
                record_num, 17,
                '平均：' + format(controlinfo.score_num / (record_num - 3), '.3f'))
            #excel_name1 = dir_path + '/' + excel_name
            print("save all results to: ", excel_name)
            book.close()
    elif args.bag:
        file = os.path.basename(args.bag)
        print('file name is', file)
        print('bag name is', args.bag)
        excel_name = os.path.splitext(args.bag)[0] + '_evaluation.xlsx'
        book = xlsxwriter.Workbook(excel_name)
        file_num = file_num + 1
        controlinfo = ControlInfo(axarr)
        controlinfo.read_bag(args.bag)
        controlinfo.plot_chassis_speed()
        sheet = controlinfo.add_excel_sheet(file)
        controlinfo.control_planning_dif(file)
        fig.canvas.mpl_connect('key_press_event', controlinfo.press)
        picture_name = os.path.splitext(args.bag)[0] + '_' + '1' + '.jpg'
        print("save picture1 to:", picture_name)
        plt.tight_layout()
        plt.savefig(picture_name, dpi=300)  #设置分辨率300
        sheet.insert_image(4, 1, picture_name)  #设置插入到Excel内，图片的缩放，默认是1
        # print trajectory
        fig, axarr = plt.subplots(1, 1)
        controlinfo = ControlInfo(axarr)
        controlinfo.read_bag(args.bag)
        controlinfo.control_planning_dif(file, trajectory_print)
        fig.canvas.mpl_connect('key_press_event', controlinfo.press)
        picture_name = os.path.splitext(args.bag)[0] + '_' + '2' + '.jpg'
        print("save picture2 to:", picture_name)
        plt.tight_layout()
        plt.savefig(picture_name, dpi=300)  #设置分辨率300
        # time.sleep(5)
        # sheet.insert_image(4, 11, picture_name) #设置插入到Excel内，图片的缩放，默认是1
        sheet.insert_image(4, 11, picture_name, {'x_scale': 0.9, 'y_scale': 1})
        print("save evaluation result to: ", excel_name)
        book.close()
    else:
        file = "real_time_file"
        file_num = file_num + 1
        controlinfo = ControlInfo(axarr)
        controlsub = node.create_reader("/apollo/control",
                                        control_cmd_pb2.ControlCommand,
                                        controlinfo.callback_control)
        canbussub = node.create_reader('/apollo/canbus/chassis',
                                       chassis_pb2.Chassis,
                                       controlinfo.callback_canbus)
        mng = plt.get_current_fig_manager()
        controlinfo.control_planning_dif(file)
        fig.canvas.mpl_connect('key_press_event', controlinfo.press)

    cyber.shutdown()
