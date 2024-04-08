#!/usr/bin/env python

###############################################################################
# Copyright 2022 The Apollo Authors. All Rights Reserved.
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
import math
import sys
import threading
import time
import os
import shutil
import xlsxwriter
import fnmatch

import matplotlib.pyplot as plt
import numpy
import tkinter.filedialog

from matplotlib import patches
from matplotlib import lines
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3.record import RecordReader

from modules.common_msgs.localization_msgs import localization_pb2
from modules.common_msgs.chassis_msgs import chassis_pb2
from modules.common_msgs.planning_msgs import planning_pb2
from modules.common_msgs.control_msgs import control_cmd_pb2


class ControlInfo(object):
    """
    ControlInfo Class
    """

    def __init__(self, axarr):
        self.imuright = []
        self.imuforward = []
        self.imuup = []
        self.heading = []
        self.controltime = []
        self.planningtime = []
        self.trajectory_type = []
        self.trajectory_gear = []
        self.localizationtime = []
        self.canbustime = []

        #station information
        self.station_reference = []
        self.current_station = []
        self.station_error = []
        self.station_error_limited = []
        #speed information
        self.speed_reference = []
        self.preview_speed_reference = []
        self.current_speed = []
        self.speed_error = []
        self.speed_offset = []
        self.speed_controller_input_limited = []

        #acceleration information
        self.acc_open = []
        self.acc_close = []
        self.slope_offset_compensation = []
        self.acceleration_lookup = []
        self.speed_lookup = []
        self.calibration_value = []
        self.acc_localization = []
        self.lon_acceleration = []

        self.vehicle_pitch = []

        self.throttlecmd = []
        self.canbus_throttlefbk = []
        self.brakecmd = []
        self.canbus_brakefbk = []
        self.gear_location = []
        self.canbus_parking_brake = []
        self.parking_brakecmd = []

        self.is_eps_online = []
        self.is_esp_online = []
        self.is_vcu_online = []

        self.is_full_stop = []
        self.is_full_stop_soft = []
        self.path_remain = []
        self.pid_saturation_status = []
        self.is_stop_reason_by_prdestrian = []
        self.is_stop_reason_by_destination = []

        self.heading_error = []
        self.lateral_error = []
        self.heading_error_rate = []
        self.lateral_error_rate = []
        self.heading_error_feedback = []
        self.lateral_error_feedback = []

        self.steercmd = []
        self.canbus_steerfbk = []
        self.canbus_speed = []
        self.curvature = []

        #debug
        self.steer_angle = []
        self.steer_angle_feedback = []
        self.steer_angle_feedforward = []
        self.steer_angle_feedback_augment = []
        self.steer_angle_lateral_contribution = []
        self.steer_angle_lateral_rate_contribution = []
        self.steer_angle_heading_contribution = []
        self.steer_angle_heading_rate_contribution = []

        self.target_speed = []
        self.target_curvature = []
        self.target_acceleration = []
        self.target_heading = []
        self.target_time = []

        self.driving_mode = 0
        self.canbus_Driving_mode = []
        self.mode_time = []
        self.gear_mode_time = []

        self.total_time_ms = []
        self.controller_time_ms = []
        self.total_time_exceeded = []

        self.planningavailable = False

        self.carx = []
        self.cary = []
        self.cartime = []
        self.planning_pathx = []
        self.planning_pathy = []
        self.planning_paths = []
        self.planning_patha = []
        self.planning_pathv = []
        self.planningx = []
        self.planningy = []
        self.is_replan = []
        self.i = 0
        self.drivingAction = []
        self.numpoints_sum = []
        self.planning_sum = []
        self.show_planning_flag = True
        self.count = 1
        self.axarr = axarr
        self.lock = threading.Lock()

    def __callback_planning(self, entity):
        """
        New Planning Trajectory
        """
        self.planning_pathx = [p.path_point.x for p in entity.trajectory_point]
        self.planning_pathy = [p.path_point.y for p in entity.trajectory_point]
        self.planning_paths = [p.path_point.s for p in entity.trajectory_point]
        self.planning_pathv = [p.v for p in entity.trajectory_point]
        self.planning_patha = [p.a for p in entity.trajectory_point]
        if self.i < 1:
            self.planningx = self.planning_pathx
            self.planningy = self.planning_pathy
        self.i += 1
        self.planningtime.append(entity.header.timestamp_sec)
        self.trajectory_type.append(entity.trajectory_type)
        self.trajectory_gear.append(entity.gear)
        self.is_replan.append(entity.is_replan)

        numpoints = len(entity.trajectory_point)
        self.numpoints_sum.append(numpoints)
        if self.show_planning_flag:
            self.add_planning_sheet(entity)
            self.count = self.count + 1

        if numpoints == 0:
            self.planningavailable = False
        else:
            self.planningavailable = True
            if self.show_planning_flag:
                self.axarr.plot(self.planning_pathx,
                                self.planning_pathy,
                                linestyle='--')
                plt.draw()

    def __callback_canbus(self, entity):
        """
        New Canbus
        """
        self.canbus_throttlefbk.append(entity.throttle_percentage)
        self.canbus_brakefbk.append(entity.brake_percentage)
        self.canbus_steerfbk.append(entity.steering_percentage)
        self.canbus_speed.append(entity.speed_mps)
        self.canbustime.append(entity.header.timestamp_sec)
        self.gear_location.append(entity.gear_location)
        self.canbus_parking_brake.append(entity.parking_brake)
        self.is_eps_online.append(entity.check_response.is_eps_online)
        self.is_esp_online.append(entity.check_response.is_esp_online)
        self.is_vcu_online.append(entity.check_response.is_vcu_online)

        if entity.gear_location == 1:
            self.gear_mode_time.append(entity.header.timestamp_sec)

        if entity.driving_mode == chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE:
            if self.driving_mode == 0:
                self.mode_time.append(entity.header.timestamp_sec)
                self.driving_mode = 1
        elif self.driving_mode == 1:
            self.mode_time.append(entity.header.timestamp_sec)
            self.driving_mode = 0
        self.canbus_Driving_mode.append(self.driving_mode)

    def __callback_localization(self, entity):
        """
        New Localization
        """
        self.imuright.append(entity.pose.linear_acceleration_vrf.x)
        self.imuforward.append(entity.pose.linear_acceleration_vrf.y)
        self.imuup.append(entity.pose.linear_acceleration_vrf.z)
        self.heading.append(entity.pose.heading)
        self.localizationtime.append(entity.header.timestamp_sec)
        heading_angle = entity.pose.heading
        acc_x = entity.pose.linear_acceleration.x
        acc_y = entity.pose.linear_acceleration.y
        acc = acc_x * math.cos(heading_angle) + acc_y * math.sin(heading_angle)
        self.acc_localization.append(acc)
        self.carx.append(entity.pose.position.x)
        self.cary.append(entity.pose.position.y)
        self.cartime.append(entity.header.timestamp_sec)

    def __callback_control(self, entity):
        """
        New Control Command
        """
        self.throttlecmd.append(entity.throttle)
        self.brakecmd.append(entity.brake)
        self.steercmd.append(entity.steering_target)
        self.parking_brakecmd.append(entity.parking_brake)
        self.controltime.append(entity.header.timestamp_sec)
        self.total_time_ms.append(entity.latency_stats.total_time_ms)

        self.acceleration_lookup.append(
            entity.debug.simple_lon_debug.acceleration_lookup)
        self.slope_offset_compensation.append(
            entity.debug.simple_lon_debug.slope_offset_compensation)
        self.speed_lookup.append(entity.debug.simple_lon_debug.speed_lookup)
        self.calibration_value.append(
            entity.debug.simple_lon_debug.calibration_value)
        self.is_full_stop.append(entity.debug.simple_lon_debug.is_full_stop)
        self.is_full_stop_soft.append(entity.debug.simple_lon_debug.is_full_stop_soft)
        self.path_remain.append(entity.debug.simple_lon_debug.path_remain)
        self.is_stop_reason_by_destination.append(entity.debug.simple_lon_debug.is_stop_reason_by_destination)
        self.is_stop_reason_by_prdestrian.append(entity.debug.simple_lon_debug.is_stop_reason_by_prdestrian)
        self.pid_saturation_status.append(
            entity.debug.simple_lon_debug.pid_saturation_status)
        self.acc_open.append(
            entity.debug.simple_lon_debug.preview_acceleration_reference)
        self.preview_speed_reference.append(
            entity.debug.simple_lon_debug.preview_speed_reference)
        self.acc_close.append(
            entity.debug.simple_lon_debug.acceleration_cmd_closeloop)
        self.station_reference.append(
            entity.debug.simple_lon_debug.station_reference)
        self.current_station.append(
            entity.debug.simple_lon_debug.current_station)
        self.station_error.append(entity.debug.simple_lon_debug.station_error)
        self.station_error_limited.append(
            entity.debug.simple_lon_debug.station_error_limited)
        self.speed_error.append(entity.debug.simple_lon_debug.speed_error)
        self.speed_offset.append(entity.debug.simple_lon_debug.speed_offset)
        self.speed_controller_input_limited.append(
            entity.debug.simple_lon_debug.speed_controller_input_limited)
        self.speed_reference.append(
            entity.debug.simple_lon_debug.speed_reference)
        self.current_speed.append(
            entity.debug.simple_lon_debug.speed_reference -
            entity.debug.simple_lon_debug.speed_error)
        self.lon_acceleration.append(entity.debug.simple_lon_debug.current_acceleration)
        self.vehicle_pitch.append(entity.debug.simple_lon_debug.vehicle_pitch)

        self.drivingAction.append(entity.pad_msg.action)
        self.curvature.append(entity.debug.simple_lat_debug.curvature * 100.0)
        self.heading_error.append(entity.debug.simple_lat_debug.heading_error)
        self.heading_error_feedback.append(
            entity.debug.simple_lat_debug.heading_error_feedback)
        self.lateral_error.append(entity.debug.simple_lat_debug.lateral_error)
        self.lateral_error_feedback.append(
            entity.debug.simple_lat_debug.lateral_error_feedback)
        self.heading_error_rate.append(
            entity.debug.simple_lat_debug.heading_error_rate)
        self.lateral_error_rate.append(
            entity.debug.simple_lat_debug.lateral_error_rate)
        self.steer_angle.append(entity.debug.simple_lat_debug.steer_angle)
        self.steer_angle_feedback.append(
            entity.debug.simple_lat_debug.steer_angle_feedback)
        self.steer_angle_feedforward.append(
            entity.debug.simple_lat_debug.steer_angle_feedforward)
        self.steer_angle_feedback_augment.append(
            entity.debug.simple_lat_debug.steer_angle_feedback_augment)
        self.steer_angle_lateral_contribution.append(
            entity.debug.simple_lat_debug.steer_angle_lateral_contribution)
        self.steer_angle_lateral_rate_contribution.append(
            entity.debug.simple_lat_debug.steer_angle_lateral_rate_contribution)
        self.steer_angle_heading_contribution.append(
            entity.debug.simple_lat_debug.steer_angle_heading_contribution)
        self.steer_angle_heading_rate_contribution.append(
            entity.debug.simple_lat_debug.steer_angle_heading_rate_contribution)

    def read_bag(self, bag_file):
        file_path = bag_file
        #bag = rosbag.Bag(file_path)
        reader = RecordReader(file_path)
        print("Begin reading the file: ", file_path)
        for msg in reader.read_messages():
            #print msg.timestamp,msg.topic
            if msg.topic == "/apollo/localization/pose":
                localization = localization_pb2.LocalizationEstimate()
                localization.ParseFromString(msg.message)
                self.__callback_localization(localization)
            elif msg.topic == "/apollo/planning":
                adc_trajectory = planning_pb2.ADCTrajectory()
                adc_trajectory.ParseFromString(msg.message)
                self.__callback_planning(adc_trajectory)
            elif msg.topic == "/apollo/control":
                control_cmd = control_cmd_pb2.ControlCommand()
                control_cmd.ParseFromString(msg.message)
                self.__callback_control(control_cmd)
            elif msg.topic == "/apollo/canbus/chassis":
                chassis = chassis_pb2.Chassis()
                chassis.ParseFromString(msg.message)
                self.__callback_canbus(chassis)
        print("Done reading the file: ", file_path)

    def plot_station_speed(self):
        fig, ax = plt.subplots(2, 1)
        ax[0].get_shared_x_axes().join(ax[0], ax[1])

        ax[0].plot(self.controltime,
                   self.station_reference,
                   label='station_reference')
        ax[0].plot(self.controltime,
                   self.current_station,
                   label='current_station')
        ax[0].plot(self.controltime, self.station_error, label='station_error')
        ax[0].plot(self.controltime,
                   self.station_error_limited,
                   label='station_error_limited')
        ax[0].plot(self.controltime, self.speed_offset, label='speed_offset')
        ax[0].legend(fontsize='medium')
        ax[0].grid(True)
        ax[0].set_title('Station information')
        ax[0].set_xlabel('Time-s')
        ax[0].set_ylabel('Station-m')

        ax[1].plot(self.controltime, self.speed_error, label='speed_error')
        ax[1].plot(self.controltime,
                   self.speed_controller_input_limited,
                   label='speed_controller_input_limited')
        ax[1].plot(self.controltime, self.speed_offset, label='speed_offset')
        ax[1].plot(self.controltime, self.current_speed, label='current_speed')
        ax[1].plot(self.controltime,
                   self.speed_reference,
                   label='speed_reference')
        ax[1].legend(fontsize='medium')
        ax[1].grid(True)
        ax[1].set_title('Speed Info')
        ax[1].set_xlabel('Time-s')
        ax[1].set_ylabel('Speed-m/s')

        if len(self.mode_time) % 2 == 1:
            self.mode_time.append(self.controltime[-1])
        for i in range(0, len(self.mode_time), 2):
            ax[0].axvspan(self.mode_time[i],
                          self.mode_time[i + 1],
                          fc='0.1',
                          alpha=0.1)
            ax[1].axvspan(self.mode_time[i],
                          self.mode_time[i + 1],
                          fc='0.1',
                          alpha=0.1)

    def plot_speed_pid(self):
        fig, ax = plt.subplots(2, 1)
        ax[0].get_shared_x_axes().join(ax[0], ax[1])

        ax[0].plot(self.controltime, self.speed_error, label='speed_error')
        ax[0].plot(self.controltime,
                   self.speed_controller_input_limited,
                   label='speed_controller_input_limited')
        ax[0].plot(self.controltime, self.speed_offset, label='speed_offset')
        ax[0].legend(fontsize='medium')
        ax[0].grid(True)
        ax[0].set_title('Speed Info')
        ax[0].set_xlabel('Time-s')
        ax[0].set_ylabel('Speed-m/s')

        ax[1].plot(self.controltime, self.acc_open, label='Acc Open')
        ax[1].plot(self.controltime,
                   self.acceleration_lookup,
                   label='Lookup Acc')
        ax[1].plot(self.controltime, self.acc_close, label='Acc Close')
        ax[1].legend(fontsize='medium')
        ax[1].grid(True)
        ax[1].set_title('Acc Info')
        ax[1].set_xlabel('Time-s')

        if len(self.mode_time) % 2 == 1:
            self.mode_time.append(self.controltime[-1])
        for i in range(0, len(self.mode_time), 2):
            ax[0].axvspan(self.mode_time[i],
                          self.mode_time[i + 1],
                          fc='0.1',
                          alpha=0.1)
            ax[1].axvspan(self.mode_time[i],
                          self.mode_time[i + 1],
                          fc='0.1',
                          alpha=0.1)

    def plot_chassis(self):
        fig, ax = plt.subplots(1, 2)
        ax[0].get_shared_x_axes().join(ax[0], ax[1])
        ax[0].plot(self.canbustime,
                   self.canbus_throttlefbk,
                   label='Throttle Feedback')
        ax[0].plot(self.controltime, self.throttlecmd, label='Throttle Command')
        ax[0].plot(self.canbustime,
                   self.canbus_brakefbk,
                   label='Brake Feedback')
        ax[0].plot(self.controltime, self.brakecmd, label='Brake Command')
        ax[0].plot(self.canbustime,
                   self.canbus_speed,
                   label='canbus speed')
        ax[0].plot(self.canbustime,
                   self.canbus_speed,
                   'ro',
                    markersize=3,
                    label='canbus speed dot')
        # ax[0].plot(self.controltime, self.is_full_stop, label='is_full_stop')
        # ax[0].plot(self.controltime, self.path_remain, label='path_remain')
        # ax[0].plot(self.canbustime, self.canbus_parking_brake, label='canbus parkingbrake')
        # ax[0].plot(self.controltime, self.parking_brakecmd, label='parkingbrake cmd')
        ax[0].legend(fontsize='medium')
        ax[0].grid(True)
        ax[0].set_title('Throttle Brake Info')
        ax[0].set_xlabel('Time-s')

        # ax[1].plot(self.controltime, self.is_full_stop_soft, label='is_full_stop_soft')
        # ax[1].plot(self.controltime,
        #            self.pid_saturation_status,
        #            label='pid_saturation_status')
        # ax[1].plot(self.controltime,
        #            self.parking_brakecmd,
        #            'o',
        #            markersize = 3,
        #            label='parkingbrake cmd dot')
        ax[1].plot(self.controltime, self.drivingAction, label='control drivingAction')
        ax[1].plot(self.canbustime,
                   self.canbus_Driving_mode,
                   label='Driving_mode')
        # ax[1].plot(self.planningtime, self.numpoints_sum,  label='numpoints')
        ax[1].plot(self.canbustime,
                   self.is_eps_online,
                   label='is_eps_online')
        # ax[1].plot(self.canbustime,
        #            self.is_eps_online,
        #            'o',
        #            markersize = 3,
        #            label='is_eps_online dot')
        ax[1].plot(self.canbustime,
                   self.is_esp_online,
                   label='is_esp_online')
        # ax[1].plot(self.canbustime,
        #            self.is_eps_online,
        #            'o',
        #            markersize = 3,
        #            label='is_eps_online dot')
        ax[1].plot(self.canbustime,
                   self.is_eps_online,
                   label='is_vcu_online')
        # ax[1].plot(self.canbustime,
        #            self.is_eps_online,
        #            'o',
        #            markersize = 3,
        #            label='is_eps_online dot')
        ax[1].legend(fontsize='medium')
        ax[1].grid(True)
        ax[1].set_title('Mode Info')
        ax[1].set_xlabel('Time-s')

        if len(self.mode_time) % 2 == 1:
            self.mode_time.append(self.controltime[-1])
        for i in range(0, len(self.mode_time), 2):
            ax[0].axvspan(self.mode_time[i],
                          self.mode_time[i + 1],
                          fc='0.1',
                          alpha=0.1)
            ax[1].axvspan(self.mode_time[i],
                          self.mode_time[i + 1],
                          fc='0.1',
                          alpha=0.1)

    def plot_full_stop(self):
        fig, ax = plt.subplots(3,1)
        ax[0].get_shared_x_axes().join(ax[0], ax[1], ax[2])
        ax[0].plot(self.controltime, self.path_remain, label='path_remain')
        ax[0].plot(self.canbustime,
                   self.canbus_parking_brake,
                   'o',
                   markersize = 3,
                   label='canbus parkingbrake dot')
        ax[0].plot(self.controltime, self.parking_brakecmd, label='parkingbrake cmd')
        ax[0].legend(fontsize='medium')
        ax[0].grid(True)
        ax[0].set_title('Epb Info')

        ax[1].plot(self.controltime, self.is_full_stop_soft, label='is_full_stop_soft')
        ax[1].plot(self.controltime,
                   self.is_full_stop,
                   'o',
                   markersize = 3,
                   label='is_full_stop')
        ax[1].plot(self.controltime,
                      self.is_stop_reason_by_destination,
                      label='is_stop_reason_by_destination')
        ax[1].plot(self.controltime,
                      self.is_stop_reason_by_prdestrian,
                      label='is_stop_reason_by_prdestrian')
        ax[1].legend(fontsize='medium')
        ax[1].set_title('Full Stop Info')
        ax[1].grid(True)

        ax[2].plot(self.canbustime,
                      self.gear_location,
                      'o',
                      markersize = 3,
                      label='gear_location dot')
        ax[2].plot(self.planningtime,
                      self.trajectory_gear,
                      label='trajectory_gear')
        ax[2].legend(fontsize='medium')
        ax[2].grid(True)
        ax[2].set_title('Gear Info')
        ax[2].set_xlabel('Time-s')

        if len(self.mode_time) % 2 == 1:
            self.mode_time.append(self.controltime[-1])
        for i in range(0, len(self.mode_time), 2):
            ax[0].axvspan(self.mode_time[i],
                          self.mode_time[i + 1],
                          fc='0.1',
                          alpha=0.1)
            ax[1].axvspan(self.mode_time[i],
                          self.mode_time[i + 1],
                          fc='0.1',
                          alpha=0.1)
            ax[2].axvspan(self.mode_time[i],
                          self.mode_time[i + 1],
                          fc='0.1',
                          alpha=0.1)

    def show_longitudinal_all(self):
        """
        Showing Longitudinal
        """
        fig, ax = plt.subplots(2, 3)
        ax[0, 0].get_shared_x_axes().join(ax[0, 0], ax[0, 0])
        ax[0, 0].get_shared_x_axes().join(ax[0, 0], ax[0, 1])
        ax[0, 0].get_shared_x_axes().join(ax[0, 0], ax[0, 2])
        ax[0, 0].get_shared_x_axes().join(ax[0, 0], ax[1, 0])
        ax[0, 0].get_shared_x_axes().join(ax[0, 0], ax[1, 1])
        ax[0, 0].get_shared_x_axes().join(ax[0, 0], ax[1, 2])

        ax[0, 0].plot(self.controltime,
                      self.current_speed,
                      label='current_speed')
        ax[0, 0].plot(self.controltime,
                      self.preview_speed_reference,
                      label='preview_speed_reference')
        ax[0, 0].plot(self.controltime,
                      self.speed_reference,
                      label='speed_reference')
        ax[0, 0].plot(self.canbustime,
                      self.canbus_speed,
                      label='canbus speed')
        ax[0, 0].legend(fontsize='medium')
        ax[0, 0].grid(True)
        ax[0, 0].set_title('Speed Info')
        ax[0, 0].set_xlabel('Time-s')

        ax[0, 1].plot(self.controltime,
                      self.station_reference,
                      label='station_reference')
        ax[0, 1].plot(self.controltime,
                      self.current_station,
                      label='current_station')
        ax[0, 1].plot(self.controltime,
                      self.station_error,
                      label='station_error')
        ax[0, 1].plot(self.controltime,
                      self.station_error_limited,
                      label='station_error_limited')
        ax[0, 1].legend(fontsize='medium')
        ax[0, 1].grid(True)
        ax[0, 1].set_title('Station information')
        ax[0, 1].set_xlabel('Time-s')
        ax[0, 1].set_ylabel('Station-m')

        ax[0, 2].plot(self.canbustime,
                      self.canbus_throttlefbk,
                      label='Throttle Feedback')
        ax[0, 2].plot(self.controltime,
                      self.throttlecmd,
                      label='Throttle Command')
        ax[0, 2].plot(self.canbustime,
                      self.canbus_brakefbk,
                      label='Brake Feedback')
        ax[0, 2].plot(self.controltime, self.brakecmd, label='Brake Command')
        ax[0, 2].legend(fontsize='medium')
        ax[0, 2].grid(True)
        ax[0, 2].set_title('Throttle Brake Info')
        ax[0, 2].set_xlabel('Time-s')

        ax[1, 0].plot(self.controltime, self.speed_error, label='speed_error')
        ax[1, 0].plot(self.controltime,
                      self.speed_controller_input_limited,
                      label='speed_controller_input_limited')
        ax[1, 0].plot(self.controltime, self.speed_offset, label='speed_offset')
        ax[1, 0].legend(fontsize='medium')
        ax[1, 0].grid(True)
        ax[1, 0].set_title('Speed Info')
        ax[1, 0].set_xlabel('Time-s')
        ax[1, 0].set_ylabel('Speed-m/s')

        ax[1, 1].plot(self.controltime, self.acc_open, label='Acc Open')
        ax[1, 1].plot(self.controltime,
                      self.acceleration_lookup,
                      label='Lookup Acc')
        ax[1, 1].plot(self.controltime, self.acc_close, label='Acc Close')
        # ax[1, 1].plot(self.cartime, self.acc_localization, label='Acc localization')
        # ax[1, 1].plot(self.controltime, self.lon_acceleration, label='Lon acceleration')
        ax[1, 1].plot(self.controltime,
                      self.slope_offset_compensation,
                      label='slope_offset_compensation')
        ax[1, 1].plot(self.controltime,
                      self.slope_offset_compensation,
                      'o',
                      markersize=1,
                      label='slope_offset_compensation dot')
        # ax[1, 1].plot(self.controltime,
        #               self.calibration_value,
        #               label='calibration_value')
        # ax[1, 1].plot(self.controltime,
        #               self.station_error,
        #               label='station_error')
        # ax[1, 1].plot(self.controltime, self.speed_error, label='speed_error')
        ax[1, 1].legend(fontsize='medium')
        ax[1, 1].grid(True)
        ax[1, 1].set_title('Acc Info')
        ax[1, 1].set_xlabel('Time-s')

        ax[1, 2].plot(self.controltime, self.is_full_stop, label='is_full_stop')
        ax[1, 2].plot(self.controltime,
                      self.is_full_stop_soft,
                      label='is_full_stop_soft')
        # ax[1, 2].plot(self.canbustime, self.canbus_parking_brake, label='canbus parkingbrake')
        ax[1, 2].plot(self.canbustime,
                      self.gear_location,
                      label='gear_location')
        ax[1, 2].plot(self.planningtime,
                      self.trajectory_gear,
                      label='trajectory_gear')
        # ax[1, 2].plot(self.controltime,
        #               self.is_stop_reason_by_destination,
        #               label='is_stop_reason_by_destination')
        # ax[1, 2].plot(self.controltime,
        #               self.is_stop_reason_by_prdestrian,
        #               label='is_stop_reason_by_prdestrian')
        # ax[1, 2].plot(self.controltime, self.path_remain, label='path_remain')
        # ax[1, 2].plot(self.controltime, self.Driving_mode, label='Driving_mode')
        # ax[1, 2].plot(self.controltime,
        #               self.drivingAction,
        #               label='drivingAction')
        # ax[1, 2].plot(self.planningtime, self.numpoints_sum, label='numpoints')
        ax[1, 2].plot(self.controltime, self.vehicle_pitch, label='vehicle_pitch')
        ax[1, 2].plot(self.controltime,
                      self.vehicle_pitch,
                      'o',
                      markersize=1,
                      label='vehicle_pitch dot')
        ax[1, 2].plot(self.controltime, self.path_remain, label='path_remain')
        ax[1, 2].plot(self.planningtime, self.is_replan, label='planning trajectory is_replan')
        ax[1, 2].plot(self.planningtime,
                      self.trajectory_type,
                      label='trajectory_type')
        ax[1, 2].legend(fontsize='medium')
        ax[1, 2].grid(True)
        ax[1, 2].set_title('Mode Info')
        ax[1, 2].set_xlabel('Time-s')

        if len(self.mode_time) % 2 == 1:
            self.mode_time.append(self.controltime[-1])
        for i in range(0, len(self.mode_time), 2):
            ax[0, 0].axvspan(self.mode_time[i],
                             self.mode_time[i + 1],
                             fc='0.1',
                             alpha=0.1)
            ax[0, 1].axvspan(self.mode_time[i],
                             self.mode_time[i + 1],
                             fc='0.1',
                             alpha=0.1)
            ax[0, 2].axvspan(self.mode_time[i],
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
            ax[1, 2].axvspan(self.mode_time[i],
                             self.mode_time[i + 1],
                             fc='0.1',
                             alpha=0.1)
        plt.draw()

    def plot_heading(self):
        """
        Plot plot_lateral_error
        """
        print("ploting Lateral Error")
        fig, ax = plt.subplots(3, 1)
        ax[0].get_shared_x_axes().join(ax[0], ax[1])
        ax[0].get_shared_x_axes().join(ax[0], ax[2])

        ax[0].plot(self.controltime, self.heading_error, label='heading_error')
        ax[0].plot(self.controltime, self.lateral_error, label='lateral_error')
        ax[0].legend(fontsize='medium')
        ax[0].grid(True)
        ax[0].set_title('Error Info')
        ax[0].set_xlabel('Time')

        ax[1].plot(self.controltime,
                   self.heading_error_rate,
                   label='heading_error_rate')
        ax[1].plot(self.controltime,
                   self.lateral_error_rate,
                   label='lateral_error_rate')
        ax[1].legend(fontsize='medium')
        ax[1].grid(True)
        ax[1].set_title('IMU Info')
        ax[1].set_xlabel('Time')

        ax[2].plot(self.canbustime,
                   self.canbus_steerfbk,
                   label='Steering Feedback')
        ax[2].plot(self.controltime, self.steercmd, label='Steering Command')
        ax[2].plot(self.controltime, self.curvature, label='Curvature')
        ax[2].legend(fontsize='medium')
        ax[2].grid(True)
        ax[2].set_title('Steering Info')
        ax[2].set_xlabel('Time-s')
        ax[2].set_xlabel('Steering-%')

        if len(self.mode_time) % 2 == 1:
            self.mode_time.append(self.controltime[-1])
        for i in range(0, len(self.mode_time), 2):
            ax[0].axvspan(self.mode_time[i],
                          self.mode_time[i + 1],
                          fc='0.1',
                          alpha=0.1)
            ax[1].axvspan(self.mode_time[i],
                          self.mode_time[i + 1],
                          fc='0.1',
                          alpha=0.1)
            ax[2].axvspan(self.mode_time[i],
                          self.mode_time[i + 1],
                          fc='0.1',
                          alpha=0.1)
        plt.draw()

    def plot_lateral_all(self):
        """
        Plot plot_lateral_all
        """
        print("ploting Lateral Error")
        fig, ax = plt.subplots(2, 3)
        ax[0, 0].get_shared_x_axes().join(ax[0, 0], ax[0, 0])
        ax[0, 0].get_shared_x_axes().join(ax[0, 0], ax[0, 1])
        ax[0, 0].get_shared_x_axes().join(ax[0, 0], ax[0, 2])
        ax[0, 0].get_shared_x_axes().join(ax[0, 0], ax[1, 0])
        ax[0, 0].get_shared_x_axes().join(ax[0, 0], ax[1, 1])
        ax[0, 0].get_shared_x_axes().join(ax[0, 0], ax[1, 2])

        ax[0, 0].plot(self.controltime,
                      self.heading_error,
                      label='heading_error')
        ax[0, 0].plot(self.controltime,
                      self.lateral_error,
                      label='lateral_error')
        ax[0, 0].plot(self.controltime,
                      self.heading_error_feedback,
                      label='heading_error_feedback')
        ax[0, 0].plot(self.controltime,
                      self.lateral_error_feedback,
                      label='lateral_error_feedback')
        ax[0, 0].legend(fontsize='medium')
        ax[0, 0].grid(True)
        ax[0, 0].set_title('Error Info')
        ax[0, 0].set_xlabel('Time')

        ax[0, 1].plot(self.controltime,
                      self.steer_angle_lateral_contribution,
                      label='steer_angle_lateral_contribution')
        ax[0, 1].plot(self.controltime,
                      self.steer_angle_lateral_rate_contribution,
                      label='steer_angle_lateral_rate_contribution')
        ax[0, 1].plot(self.controltime,
                      self.steer_angle_heading_contribution,
                      label='steer_angle_heading_contribution')
        ax[0, 1].plot(self.controltime,
                      self.steer_angle_heading_rate_contribution,
                      label='steer_angle_heading_rate_contribution')
        ax[0, 1].legend(fontsize='medium')
        ax[0, 1].grid(True)
        ax[0, 1].set_title('IMU Info')
        ax[0, 1].set_xlabel('Time')

        ax[1, 0].plot(self.canbustime,
                      self.canbus_steerfbk,
                      label='Steering Feedback')
        ax[1, 0].plot(self.controltime, self.steercmd, label='Steering Command')
        ax[1, 0].plot(self.controltime, self.curvature, label='Curvature')
        ax[1, 0].legend(fontsize='medium')
        ax[1, 0].grid(True)
        ax[1, 0].set_title('Steering Info')
        ax[1, 0].set_xlabel('Time-s')
        ax[1, 0].set_xlabel('Steering-%')

        ax[1, 1].plot(self.controltime, self.steer_angle, label='steer_angle')
        ax[1, 1].plot(self.controltime,
                      self.steer_angle_feedback,
                      label='steer_angle_feedback')
        ax[1, 1].plot(self.controltime,
                      self.steer_angle_feedforward,
                      label='steer_angle_feedforward')
        ax[1, 1].plot(self.controltime,
                      self.steer_angle_feedback_augment,
                      label='steer_angle_feedback_augment')
        ax[1, 1].legend(fontsize='medium')
        ax[1, 1].grid(True)
        ax[1, 1].set_title('steer_angle Info')
        ax[1, 1].set_xlabel('Time-s')
        ax[1, 1].set_xlabel('Steering-%')

        ax[0, 2].plot(self.cartime,
                      self.carx,
                      label='Pose X')
        ax[0, 2].plot(self.cartime,
                      self.carx,
                      'ro',
                      markersize=1,
                      label='Pose X dot')
        ax[0, 2].legend(fontsize='medium')
        ax[0, 2].grid(True)
        ax[0, 2].set_title('Localization Info X')
        ax[0, 2].set_xlabel('Time')


        ax[1, 2].plot(self.cartime,
                      self.cary,
                      label='Pose Y')
        ax[1, 2].plot(self.cartime,
                      self.cary,
                      'ro',
                      markersize=1,
                      label='Pose X dot')
        ax[1, 2].legend(fontsize='medium')
        ax[1, 2].grid(True)
        ax[1, 2].set_title('Localization Info Y')
        ax[1, 2].set_xlabel('Time')

        if len(self.mode_time) % 2 == 1:
            self.mode_time.append(self.controltime[-1])
        for i in range(0, len(self.mode_time), 2):
            ax[0, 0].axvspan(self.mode_time[i],
                             self.mode_time[i + 1],
                             fc='0.1',
                             alpha=0.1)
            ax[0, 1].axvspan(self.mode_time[i],
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
            ax[0, 2].axvspan(self.mode_time[i],
                             self.mode_time[i + 1],
                             fc='0.1',
                             alpha=0.1)
            ax[1, 2].axvspan(self.mode_time[i],
                             self.mode_time[i + 1],
                             fc='0.1',
                             alpha=0.1)
        plt.draw()

    def plot_steering(self):
        """
        Plot plot_heading_error
        """
        print("ploting Heading Error")
        fig, ax = plt.subplots(2, 1)

        ax[0].plot(self.canbustime,
                   self.canbus_steerfbk,
                   label='Steering Feedback')
        ax[0].plot(self.controltime, self.steercmd, label='Steering Command')
        ax[0].plot(self.controltime, self.curvature, label='Curvature')
        ax[0].legend(fontsize='medium')
        ax[0].grid(True)
        ax[0].set_title('Steering Info')
        ax[0].set_xlabel('Time-s')
        ax[0].set_xlabel('Steering-%')

        ax[1].plot(self.speed_lookup,
                   self.acceleration_lookup,
                   label='Table Lookup')
        ax[1].plot(self.target_speed, self.target_acceleration, label='Target')
        ax[1].legend(fontsize='medium')
        ax[1].grid(True)
        ax[1].set_title('Calibration Lookup')
        ax[1].set_xlabel('Speed')
        ax[1].set_ylabel('Acceleration')

        if len(self.mode_time) % 2 == 1:
            self.mode_time.append(self.controltime[-1])
        for i in range(0, len(self.mode_time), 2):
            ax[0].axvspan(self.mode_time[i],
                          self.mode_time[i + 1],
                          fc='0.1',
                          alpha=0.1)
            ax[1].axvspan(self.mode_time[i],
                          self.mode_time[i + 1],
                          fc='0.1',
                          alpha=0.1)
        plt.draw()

    def plot_localization(self):
        """
        Plot localization
        """
        print("ploting Localization")
        self.axarr.plot(self.planningx,
                        self.planningy,
                        color='blue',
                        linestyle='--',
                        label='planning path')
        self.axarr.plot(self.carx,
                        self.cary,
                        color='red',
                        label='localization pose')
        self.axarr.plot(self.carx,
                        self.cary,
                        'ro',
                        markersize=5,
                        label='localization pose dot')
        self.axarr.plot(self.planning_pathx,
                        self.planning_pathy,
                        color='green',
                        label='planning path remain')
        legend = self.axarr.legend(fontsize='small')
        frame = legend.get_frame()
        frame.set_alpha(1)
        frame.set_facecolor('none')
        self.axarr.grid(True)
        self.axarr.set_title('Trajectory')
        self.axarr.set_xlabel('x')
        self.axarr.set_ylabel('y')
        self.axarr.axis('equal')
        plt.draw()

    def add_longitudinal_sheet(self):
        sheet = book.add_worksheet('longitudinal')

        # 设置单元格居中，自动换行
        book_format = book.add_format({'align': 'center', 'text_wrap': True})
        sheet.write(0, 0, 'controltime', book_format)
        sheet.write(0, 1, 'canbustime', book_format)
        sheet.write(0, 2, 'current_speed', book_format)
        sheet.write(0, 3, 'speed_reference', book_format)
        sheet.write(0, 4, 'current_station', book_format)
        sheet.write(0, 5, 'station_reference', book_format)
        sheet.write(0, 6, 'station_error_limited', book_format)
        sheet.write(0, 7, 'speed_error', book_format)
        sheet.write(0, 8, 'speed_offset', book_format)
        sheet.write(0, 9, 'speed_controller_input_limited', book_format)
        sheet.write(0, 10, 'acc_open', book_format)
        sheet.write(0, 11, 'acc_close', book_format)
        sheet.write(0, 12, 'acceleration_lookup', book_format)
        sheet.write(0, 13, 'throttlecmd', book_format)
        sheet.write(0, 14, 'throttlefbk', book_format)
        sheet.write(0, 15, 'brakecmd', book_format)
        sheet.write(0, 16, 'brakefbk', book_format)
        sheet.write(0, 17, 'is_full_stop', book_format)
        sheet.write(0, 18, 'pid_saturation_status', book_format)
        sheet.write(0, 19, 'drivingAction', book_format)
        sheet.write(0, 20, 'path_remain', book_format)

        sheet.write_column(1, 0, self.controltime, book_format)
        sheet.write_column(1, 1, self.canbustime, book_format)
        sheet.write_column(1, 2, self.current_speed, book_format)
        sheet.write_column(1, 3, self.speed_reference, book_format)
        sheet.write_column(1, 4, self.current_station, book_format)
        sheet.write_column(1, 5, self.station_reference, book_format)
        sheet.write_column(1, 6, self.station_error_limited, book_format)
        sheet.write_column(1, 7, self.speed_error, book_format)
        sheet.write_column(1, 8, self.speed_offset, book_format)
        sheet.write_column(1, 9, self.speed_controller_input_limited,
                           book_format)
        sheet.write_column(1, 10, self.acc_open, book_format)
        sheet.write_column(1, 11, self.acc_close, book_format)
        sheet.write_column(1, 12, self.acceleration_lookup, book_format)
        sheet.write_column(1, 13, self.throttlecmd, book_format)
        sheet.write_column(1, 14, self.canbus_throttlefbk, book_format)
        sheet.write_column(1, 15, self.brakecmd, book_format)
        sheet.write_column(1, 16, self.canbus_brakefbk, book_format)
        sheet.write_column(1, 17, self.is_full_stop, book_format)
        sheet.write_column(1, 18, self.pid_saturation_status, book_format)
        sheet.write_column(1, 19, self.drivingAction, book_format)
        sheet.write_column(1, 17, self.path_remain, book_format)

        # 设置0行行高
        sheet.set_row(0, 45)
        sheet.set_row(1, 25)
        # 设置0列每个单元格宽度
        sheet.set_column(0, 0, 12)
        # 设置1-19列每个单元格宽度
        sheet.set_column(1, 16, 8)
        sheet.set_column(17, 19, 7)

    def add_lateral_sheet(self):
        sheet = book.add_worksheet('lateral')

        # 设置单元格居中，自动换行
        book_format = book.add_format({'align': 'center', 'text_wrap': True})
        sheet.write(0, 0, 'controltime', book_format)
        sheet.write(0, 1, 'canbustime', book_format)
        sheet.write(0, 2, 'heading_error', book_format)
        sheet.write(0, 3, 'lateral_error', book_format)
        sheet.write(0, 4, 'heading_error_rate', book_format)
        sheet.write(0, 5, 'lateral_error_rate', book_format)
        sheet.write(0, 6, 'steercmd', book_format)
        sheet.write(0, 7, 'steerfbk', book_format)
        sheet.write(0, 8, 'curvature', book_format)

        sheet.write_column(1, 0, self.controltime, book_format)
        sheet.write_column(1, 1, self.canbustime, book_format)
        sheet.write_column(1, 2, self.heading_error, book_format)
        sheet.write_column(1, 3, self.lateral_error, book_format)
        sheet.write_column(1, 4, self.heading_error_rate, book_format)
        sheet.write_column(1, 5, self.lateral_error_rate, book_format)
        sheet.write_column(1, 6, self.steercmd, book_format)
        sheet.write_column(1, 7, self.canbus_steerfbk, book_format)
        sheet.write_column(1, 8, self.curvature, book_format)

        # 设置0行行高
        sheet.set_row(0, 45)
        sheet.set_row(1, 25)
        # 设置0列每个单元格宽度
        sheet.set_column(0, 0, 12)
        # 设置1-19列每个单元格宽度
        sheet.set_column(1, 16, 8)
        sheet.set_column(17, 19, 7)

    def add_planning_sheet(self, entity):
        # 设置单元格居中，自动换行
        book_format = book.add_format({'align': 'center', 'text_wrap': True})
        pathx_sheet.write_row(self.count, 1, self.planning_pathx, book_format)
        pathy_sheet.write_row(self.count, 1, self.planning_pathy, book_format)
        paths_sheet.write_row(self.count, 1, self.planning_paths, book_format)
        patha_sheet.write_row(self.count, 1, self.planning_patha, book_format)
        pathv_sheet.write_row(self.count, 1, self.planning_pathv, book_format)

        pathx_sheet.write(self.count, 0, entity.header.timestamp_sec,
                          book_format)
        pathy_sheet.write(self.count, 0, entity.header.timestamp_sec,
                          book_format)
        paths_sheet.write(self.count, 0, entity.header.timestamp_sec,
                          book_format)
        patha_sheet.write(self.count, 0, entity.header.timestamp_sec,
                          book_format)
        pathv_sheet.write(self.count, 0, entity.header.timestamp_sec,
                          book_format)

    def add_localization_sheet(self):
        sheet = book.add_worksheet('localization')

        # 设置单元格居中，自动换行
        book_format = book.add_format({'align': 'center', 'text_wrap': True})
        sheet.write(0, 0, 'cartime', book_format)
        sheet.write(0, 1, 'carx', book_format)
        sheet.write(0, 2, 'cary', book_format)

        sheet.write_column(1, 0, self.cartime, book_format)
        sheet.write_column(1, 1, self.carx, book_format)
        sheet.write_column(1, 2, self.cary, book_format)

    def show_longitudinal(self):
        """
        ploting Longitudinal
        """
        self.plot_station_speed()
        self.plot_speed_pid()
        self.plot_chassis()
        self.add_longitudinal_sheet()

        # plt.show()

    def show_lateral(self):
        """
        Plot everything in time domain
        """
        print("ploting Lateral")
        self.plot_lateral_all()
        # self.plot_heading()
        # self.plot_steering()
        self.add_lateral_sheet()
        # plt.show()

    def show_localization(self):
        self.plot_localization()
        self.add_localization_sheet()

    def press(self, event):
        """
        Keyboard events during plotting
        """
        if event.key == 'q' or event.key == 'Q':
            plt.close('all')

    def Print_len(self):
        """
        Print_len
        """
        print("Print_len")
        print(str(len(self.controltime)) + "-controltime")
        print(str(len(self.planningtime)) + "-planningtime")
        print(str(len(self.localizationtime)) + "-localizationtime")
        print(str(len(self.canbustime)) + "-canbustime")
        print(str(len(self.station_reference)) + "-station_reference")
        print(str(len(self.current_station)) + "-current_station")
        print(str(len(self.station_error)) + "-station_error")
        print(str(len(self.station_error_limited)) + "-station_error_limited")
        print(str(len(self.speed_reference)) + "-speed_reference")
        print(str(len(self.current_speed)) + "-current_speed")
        print(str(len(self.speed_error)) + "-speed_error")
        print(str(len(self.speed_offset)) + "-speed_offset")
        print(
            str(len(self.speed_controller_input_limited)) +
            "-speed_controller_input_limited")
        print(str(len(self.acc_open)) + "-acc_open")
        print(str(len(self.acc_close)) + "-acc_close")
        print(
            str(len(self.slope_offset_compensation)) +
            "-slope_offset_compensation")
        print(str(len(self.acceleration_lookup)) + "-acceleration_lookup")
        print(str(len(self.speed_lookup)) + "-speed_lookup")
        print(str(len(self.calibration_value)) + "-calibration_value")
        print(str(len(self.throttlecmd)) + "-throttlecmd")
        print(str(len(self.canbus_throttlefbk)) + "-throttlefbk")
        print(str(len(self.brakecmd)) + "-brakecmd")
        print(str(len(self.canbus_brakefbk)) + "-brakefbk")
        print(str(len(self.is_full_stop)) + "-is_full_stop")
        print(str(len(self.pid_saturation_status)) + "-pid_saturation_status")
        print(str(len(self.heading_error)) + "-heading_error")
        print(str(len(self.lateral_error)) + "-lateral_error")
        print(str(len(self.heading_error_rate)) + "-heading_error_rate")
        print(str(len(self.steercmd)) + "-steercmd")
        print(str(len(self.canbus_steerfbk)) + "-steerfbk")
        print(str(len(self.canbus_speed)) + "-speed")
        print(str(len(self.curvature)) + "-curvature")
        print(str(len(self.carx)) + "-carx")
        print(str(len(self.cary)) + "-cary")
        print(str(len(self.cartime)) + "-cartime")
        print(str(len(self.planning_pathx)) + "-planning_pathx")
        print(str(len(self.planning_pathy)) + "-planning_pathy")
        print(str(len(self.planningx)) + "-planningx")
        print(str(len(self.planningy)) + "-planningy")
        print(str(len(self.drivingAction)) + "-drivingAction")
        print(str(len(self.canbus_Driving_mode)) + "-Driving_mode")
        print(str(len(self.trajectory_type)) + "-trajectory_type")
        print(str(len(self.path_remain)) + "-path_remain")
        print(str(len(self.acc_localization)) + "-acc_localization")
        print(str(len(self.gear_mode_time)) + "-gear_mode_time")
        print(str(len(self.mode_time)) + "-mode_time")


def is_record_file(path):
    """Naive check if a path is a record."""
    return path.endswith('.record') or \
                fnmatch.fnmatch(path, '*.record.?????') or \
                    fnmatch.fnmatch(path, '*.record.*') or \
                        fnmatch.fnmatch(path, '*.record.?????.*')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Process and analyze control and planning data')
    parser.add_argument('--bag', type=str, help='use Rosbag')
    parser.add_argument('--path', type=str, help='path for bag files')
    args = parser.parse_args()
    fig, axarr = plt.subplots()
    controlinfo = ControlInfo(axarr)
    current_dir = sys.path[0]
    print("current dir is", current_dir)

    if args.path:
        dir_path = args.path
        #create datafile dir
        datafile_path = args.path + "/data_dir"
        isExists = os.path.exists(datafile_path)
        if not isExists:
            os.makedirs(datafile_path)
            print('Create success the file:', datafile_path)
        else:
            print('The file already exists:', datafile_path)
            shutil.rmtree(datafile_path)
            os.makedirs(datafile_path)
            print(datafile_path + 'recreate success')

        #create datafile
        excel_name = 'data.xlsx'
        book = xlsxwriter.Workbook(datafile_path + '/' + excel_name)
        book_format = book.add_format({'align': 'center', 'text_wrap': True})

        #add_worksheet
        if controlinfo.show_planning_flag:
            pathx_sheet = book.add_worksheet('planning_pathx')
            pathy_sheet = book.add_worksheet('planning_pathy')
            paths_sheet = book.add_worksheet('planning_paths')
            patha_sheet = book.add_worksheet('planning_patha')
            pathv_sheet = book.add_worksheet('planning_pathv')
            pathx_sheet.write(0, 0, "timestamp_sec", book_format)
            pathy_sheet.write(0, 0, "timestamp_sec", book_format)
            paths_sheet.write(0, 0, "timestamp_sec", book_format)
            patha_sheet.write(0, 0, "timestamp_sec", book_format)
            pathv_sheet.write(0, 0, "timestamp_sec", book_format)

        if os.path.isdir(dir_path):
            dirs = os.listdir(dir_path)
            dirs.sort()
            for file in dirs:
                #print "os.path.splitext(file)[1]", os.path.splitext(file)[1]
                if is_record_file(file):
                    controlinfo.read_bag(dir_path + file)
    elif args.bag:
        #create datafile dir
        datafile_path = args.bag + "_dir"
        isExists = os.path.exists(datafile_path)
        if not isExists:
            os.makedirs(datafile_path)
            print('Create success the file:', datafile_path)
        else:
            print('The file already exists:', datafile_path)
            shutil.rmtree(datafile_path)
            os.makedirs(datafile_path)
            print(datafile_path + 'recreate success')

        #create datafile
        excel_name = 'data.xlsx'
        book = xlsxwriter.Workbook(datafile_path + '/' + excel_name)
        book_format = book.add_format({'align': 'center', 'text_wrap': True})

        #add_worksheet
        if controlinfo.show_planning_flag:
            pathx_sheet = book.add_worksheet('planning_pathx')
            pathy_sheet = book.add_worksheet('planning_pathy')
            paths_sheet = book.add_worksheet('planning_paths')
            patha_sheet = book.add_worksheet('planning_patha')
            pathv_sheet = book.add_worksheet('planning_pathv')
            pathx_sheet.write(0, 0, "timestamp_sec", book_format)
            pathy_sheet.write(0, 0, "timestamp_sec", book_format)
            paths_sheet.write(0, 0, "timestamp_sec", book_format)
            patha_sheet.write(0, 0, "timestamp_sec", book_format)
            pathv_sheet.write(0, 0, "timestamp_sec", book_format)

        controlinfo.read_bag(args.bag)
    else:
        #create datafile
        excel_name = 'data.xlsx'
        book = xlsxwriter.Workbook(excel_name)
        book_format = book.add_format({'align': 'center', 'text_wrap': True})

    controlinfo.Print_len()
    # controlinfo.show_longitudinal()
    controlinfo.show_lateral()
    controlinfo.show_localization()
    controlinfo.show_longitudinal_all()
    controlinfo.plot_chassis()
    controlinfo.plot_full_stop()
    book.close()

    fig.canvas.mpl_connect('key_press_event', controlinfo.press)
    plt.show()
