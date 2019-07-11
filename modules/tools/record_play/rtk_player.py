#!/usr/bin/env python

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
Generate Planning Path
"""

import argparse
import atexit
import logging
import os
import sys
import time
import math

from cyber_py import cyber
from cyber_py import cyber_time
import scipy.signal as signal
from logger import Logger
from numpy import genfromtxt

from modules.canbus.proto import chassis_pb2
from modules.common.proto import pnc_point_pb2
from modules.common.proto import drive_state_pb2
from modules.control.proto import pad_msg_pb2
from modules.localization.proto import localization_pb2
from modules.planning.proto import planning_pb2
from modules.common.configs.proto import vehicle_config_pb2
import common.proto_utils as proto_utils

APOLLO_ROOT = os.path.join(os.path.dirname(__file__), '../../../')
SEARCH_INTERVAL = 5000
CHANGE_TO_COM = False


class RtkPlayer(object):
    """
    rtk player class
    """

    def __init__(self, record_file, node, speedmultiplier, completepath,
                 replan):
        """Init player."""
        self.firstvalid = False
        self.logger = Logger.get_logger(tag="RtkPlayer")
        self.logger.info("Load record file from: %s" % record_file)
        try:
            file_handler = open(record_file, 'r')
        except IOError:
            self.logger.error("Cannot find file: " + record_file)
            file_handler.close()
            sys.exit(0)

        self.data = genfromtxt(file_handler, delimiter=',', names=True)
        file_handler.close()

        self.localization = localization_pb2.LocalizationEstimate()
        self.chassis = chassis_pb2.Chassis()
        self.padmsg = pad_msg_pb2.PadMessage()
        self.localization_received = False
        self.chassis_received = False

        self.planning_pub = node.create_writer('/apollo/planning',
                                               planning_pb2.ADCTrajectory)

        self.speedmultiplier = speedmultiplier / 100
        self.terminating = False
        self.sequence_num = 0

        b, a = signal.butter(6, 0.05, 'low')
        self.data['acceleration'] = signal.filtfilt(b, a,
                                                    self.data['acceleration'])

        self.start = 0
        self.end = 0
        self.closestpoint = 0
        self.automode = False

        self.replan = (replan == 't')
        self.completepath = (completepath == 't')

        self.estop = False
        self.logger.info("Planning Ready")

        vehicle_config = vehicle_config_pb2.VehicleConfig()
        proto_utils.get_pb_from_text_file(
            "/apollo/modules/common/data/vehicle_param.pb.txt", vehicle_config)
        self.vehicle_param = vehicle_config.vehicle_param

    def localization_callback(self, data):
        """
        New localization Received
        """
        self.localization.CopyFrom(data)
        self.carx = self.localization.pose.position.x
        self.cary = self.localization.pose.position.y
        self.carz = self.localization.pose.position.z
        self.localization_received = True

    def chassis_callback(self, data):
        """
        New chassis Received
        """
        self.chassis.CopyFrom(data)
        self.automode = (self.chassis.driving_mode ==
                         chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE)
        self.chassis_received = True

    def padmsg_callback(self, data):
        """
        New message received
        """
        if self.terminating is True:
            self.logger.info("terminating when receive padmsg")
            return

        self.padmsg.CopyFrom(data)

    def restart(self):
        self.logger.info("before replan self.start=%s, self.closestpoint=%s" %
                         (self.start, self.closestpoint))

        self.closestpoint = self.closest_dist()
        self.logger.debug("replan!")
        self.start = max(self.closestpoint - 1, 0)
        self.logger.debug("replan_start:", self.start)
        self.starttime = cyber_time.Time.now().to_sec()
        self.logger.debug("at time", self.starttime)
        # self.end = min(self.start + 1000, len(self.data) - 1)
        self.end = self.next_gear_switch_time(self.start, len(self.data))
        self.logger.debug("replan_end:", self.end)

        self.logger.info("finish replan at time %s, self.closestpoint=%s" %
                         (self.starttime, self.closestpoint))

    def closest_dist(self):
        shortest_dist_sqr = float('inf')
        self.logger.info("before closest self.start=%s" % (self.start))
        self.logger.debug("before closest self.start=%s" % (self.start))
        search_start = max(self.start - SEARCH_INTERVAL / 2, 0)
        search_end = min(self.start + SEARCH_INTERVAL / 2, len(self.data))
        self.logger.debug("search_start:", search_start)
        self.logger.debug("search_end:", search_end)
        start = self.start
        self.logger.debug("self.start:", self.start)
        for i in range(search_start, search_end):
            dist_sqr = (self.carx - self.data['x'][i]) ** 2 + \
                (self.cary - self.data['y'][i]) ** 2
            if dist_sqr <= shortest_dist_sqr and self.data['gear'][i] == self.chassis.gear_location:
                start = i
                shortest_dist_sqr = dist_sqr
        return start

    def closest_time(self):
        time_elapsed = cyber_time.Time.now().to_sec() - self.starttime
        closest_time = self.start
        time_diff = self.data['time'][closest_time] - \
            self.data['time'][self.closestpoint]

        while time_diff < time_elapsed and closest_time < (len(self.data) - 1):
            closest_time = closest_time + 1
            time_diff = self.data['time'][closest_time] - \
                self.data['time'][self.closestpoint]

        return closest_time

    def next_gear_switch_time(self, start, end):
        for i in range(start, end):
            # trajectory with gear switch
            # include gear_neutral at the beginning of a trajectory
            if((self.data['gear'][i] == 1 or self.data['gear'][i] == 2)
                    and (self.data['gear'][i + 1] != self.data['gear'][i]) ):
                self.logger.debug("enter i in while loop: [ %s ]" % i)
                self.logger.debug("self.data['gear'][i] != 1: %s" % self.data['gear'][i])
                self.logger.debug("self.data['gear'][i] != 2: %s" % self.data['gear'][i])
                # find next gear = 1 or 2
                i += 1
                while i < end and (self.data['gear'][i] != 1) and (self.data['gear'][i] != 2):
                    i += 1
                self.logger.debug("i in while loop: [ %s ]" % i)
                return i - 1
        # trajectory without gear switch
        self.logger.debug("i at end: [ %s ]" % i)
        return min(i, end - 1)

    def publish_planningmsg(self):
        """
        Generate New Path
        """
        if not self.localization_received:
            self.logger.warning(
                "locaization not received yet when publish_planningmsg")
            return

        planningdata = planning_pb2.ADCTrajectory()
        now = cyber_time.Time.now().to_sec()
        planningdata.header.timestamp_sec = now
        planningdata.header.module_name = "planning"
        planningdata.header.sequence_num = self.sequence_num
        self.sequence_num = self.sequence_num + 1

        self.logger.debug(
            "publish_planningmsg: before adjust start: self.start=%s, self.end=%s"
            % (self.start, self.end))
        if self.replan or self.sequence_num <= 1 or not self.automode:
            self.logger.info(
                "trigger replan: self.replan=%s, self.sequence_num=%s, self.automode=%s"
                % (self.replan, self.sequence_num, self.automode))
            self.restart()
        else:
            timepoint = self.closest_time()
            distpoint = self.closest_dist()

            if self.data['gear'][timepoint] == self.data['gear'][distpoint]:
                self.start = max(min(timepoint, distpoint), 0)
            elif self.data['gear'][timepoint] == self.chassis.gear_location:
                self.start = timepoint
            else:
                self.start = distpoint

            self.logger.debug("timepoint:[%s]" % timepoint)
            self.logger.debug("distpoint:[%s]" % distpoint)
            self.logger.debug(
                "trajectory start point: [%s], gear is [%s]" % (self.start, self.data['gear'][self.start]))

            self.end = self.next_gear_switch_time(self.start, len(self.data))
            self.logger.debug("len of data: ", len(self.data))
            self.logger.debug("trajectory end point: [%s], gear is [%s]" %
                              (self.end, self.data['gear'][self.end]))

            xdiff_sqr = (self.data['x'][timepoint] - self.carx)**2
            ydiff_sqr = (self.data['y'][timepoint] - self.cary)**2
            if xdiff_sqr + ydiff_sqr > 4.0:
                self.logger.info("trigger replan: distance larger than 2.0")
                self.restart()

        if self.completepath:
            self.start = 0
            self.end = len(self.data) - 1

        self.logger.debug(
            "publish_planningmsg: after adjust start: self.start=%s, self.end=%s"
            % (self.start, self.end))

        planningdata.total_path_length = self.data['s'][self.end] - \
            self.data['s'][self.start]
        self.logger.info("total number of planning data point: %d" % (self.end - self.start))
        planningdata.total_path_time = self.data['time'][self.end] - \
            self.data['time'][self.start]
        planningdata.gear = int(self.data['gear'][self.closest_time()])
        planningdata.engage_advice.advice = \
            drive_state_pb2.EngageAdvice.READY_TO_ENGAGE

        for i in range(self.start, self.end):
            adc_point = pnc_point_pb2.TrajectoryPoint()
            adc_point.path_point.x = self.data['x'][i]
            adc_point.path_point.y = self.data['y'][i]
            adc_point.path_point.z = self.data['z'][i]
            adc_point.v = self.data['speed'][i] * self.speedmultiplier
            adc_point.a = self.data['acceleration'][i] * self.speedmultiplier
            adc_point.path_point.kappa = self.data['curvature'][i]
            adc_point.path_point.dkappa = self.data['curvature_change_rate'][i]
            adc_point.path_point.theta = self.data['theta'][i]
            adc_point.path_point.s = self.data['s'][i]

            if CHANGE_TO_COM:
                # translation vector length(length / 2 - back edge to center)
                adc_point.path_point.x = adc_point.path_point.x + \
                    (self.vehicle_param.length / 2 - self.vehicle_param.back_edge_to_center) * \
                    math.cos(adc_point.path_point.theta)
                adc_point.path_point.y = adc_point.path_point.y + \
                    (self.vehicle_param.length / 2 - self.vehicle_param.back_edge_to_center) * \
                    math.sin(adc_point.path_point.theta)

            if planningdata.gear == chassis_pb2.Chassis.GEAR_REVERSE:
                adc_point.v = -adc_point.v
                adc_point.path_point.s = -adc_point.path_point.s

            time_diff = self.data['time'][i] - \
                self.data['time'][self.closestpoint]

            adc_point.relative_time = time_diff / self.speedmultiplier - (
                now - self.starttime)

            planningdata.trajectory_point.extend([adc_point])

        planningdata.estop.is_estop = self.estop

        self.planning_pub.write(planningdata)
        self.logger.debug("Generated Planning Sequence: " +
                          str(self.sequence_num - 1))

    def shutdown(self):
        """
        shutdown cyber
        """
        self.terminating = True
        self.logger.info("Shutting Down...")
        time.sleep(0.2)

    def quit(self, signum, frame):
        """
        shutdown the keypress thread
        """
        sys.exit(0)


def main():
    """
    Main cyber
    """
    parser = argparse.ArgumentParser(
        description='Generate Planning Trajectory from Data File')
    parser.add_argument(
        '-s',
        '--speedmulti',
        help='Speed multiplier in percentage (Default is 100) ',
        type=float,
        default='100')
    parser.add_argument(
        '-c', '--complete', help='Generate complete path (t/F)', default='F')
    parser.add_argument(
        '-r',
        '--replan',
        help='Always replan based on current position(t/F)',
        default='F')
    args = vars(parser.parse_args())

    node = cyber.Node("rtk_player")

    Logger.config(
        log_file=os.path.join(APOLLO_ROOT, 'data/log/rtk_player.log'),
        use_stdout=True,
        log_level=logging.DEBUG)

    record_file = os.path.join(APOLLO_ROOT, 'data/log/garage.csv')

    player = RtkPlayer(record_file, node, args['speedmulti'],
                       args['complete'].lower(), args['replan'].lower())
    atexit.register(player.shutdown)

    node.create_reader('/apollo/canbus/chassis', chassis_pb2.Chassis,
                       player.chassis_callback)

    node.create_reader('/apollo/localization/pose',
                       localization_pb2.LocalizationEstimate,
                       player.localization_callback)

    node.create_reader('/apollo/control/pad', pad_msg_pb2.PadMessage,
                       player.padmsg_callback)

    while not cyber.is_shutdown():
        now = cyber_time.Time.now().to_sec()
        player.publish_planningmsg()
        sleep_time = 0.1 - (cyber_time.Time.now().to_sec() - now)
        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == '__main__':
    cyber.init()
    main()
    cyber.shutdown()
