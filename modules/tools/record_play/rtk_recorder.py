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
Record GPS and IMU data
"""

import atexit
import logging
import math
import os
import sys
import time

from cyber.python.cyber_py3 import cyber
from gflags import FLAGS

from modules.tools.common.logger import Logger
import modules.tools.common.proto_utils as proto_utils
from modules.canbus.proto import chassis_pb2
from modules.common.configs.proto import vehicle_config_pb2
from modules.localization.proto import localization_pb2


class RtkRecord(object):
    """
    rtk recording class
    """

    def write(self, data):
        """Wrap file write function to flush data to disk"""
        self.file_handler.write(data)
        self.file_handler.flush()

    def __init__(self, record_file):
        self.firstvalid = False
        self.logger = Logger.get_logger("RtkRecord")
        self.record_file = record_file
        self.logger.info("Record file to: " + record_file)

        try:
            self.file_handler = open(record_file, 'w')
        except IOError:
            self.logger.error("Open file %s failed" % (record_file))
            self.file_handler.close()
            sys.exit(1)

        self.write("x,y,z,speed,acceleration,curvature,"
                   "curvature_change_rate,time,theta,gear,s,throttle,brake,steering\n")

        self.localization = localization_pb2.LocalizationEstimate()
        self.chassis = chassis_pb2.Chassis()
        self.chassis_received = False

        self.cars = 0.0
        self.startmoving = False

        self.terminating = False
        self.carcurvature = 0.0

        self.prev_carspeed = 0.0

        vehicle_config = vehicle_config_pb2.VehicleConfig()
        proto_utils.get_pb_from_text_file(
            "/apollo/modules/common/data/vehicle_param.pb.txt", vehicle_config)
        self.vehicle_param = vehicle_config.vehicle_param

    def chassis_callback(self, data):
        """
        New message received
        """
        if self.terminating is True:
            self.logger.info("terminating when receive chassis msg")
            return

        self.chassis.CopyFrom(data)
        #self.chassis = data
        if math.isnan(self.chassis.speed_mps):
            self.logger.warning("find nan speed_mps: %s" % str(self.chassis))
        if math.isnan(self.chassis.steering_percentage):
            self.logger.warning(
                "find nan steering_percentage: %s" % str(self.chassis))
        self.chassis_received = True

    def localization_callback(self, data):
        """
        New message received
        """
        if self.terminating is True:
            self.logger.info("terminating when receive localization msg")
            return

        if not self.chassis_received:
            self.logger.info(
                "chassis not received when localization is received")
            return

        self.localization.CopyFrom(data)
        #self.localization = data
        carx = self.localization.pose.position.x
        cary = self.localization.pose.position.y
        carz = self.localization.pose.position.z
        cartheta = self.localization.pose.heading
        if math.isnan(self.chassis.speed_mps):
            self.logger.warning("find nan speed_mps: %s" % str(self.chassis))
            return
        if math.isnan(self.chassis.steering_percentage):
            self.logger.warning(
                "find nan steering_percentage: %s" % str(self.chassis))
            return
        carspeed = self.chassis.speed_mps
        caracceleration = self.localization.pose.linear_acceleration_vrf.y

        speed_epsilon = 1e-9
        if abs(self.prev_carspeed) < speed_epsilon \
                and abs(carspeed) < speed_epsilon:
            caracceleration = 0.0

        carsteer = self.chassis.steering_percentage
        carmax_steer_angle = self.vehicle_param.max_steer_angle
        carsteer_ratio = self.vehicle_param.steer_ratio
        carwheel_base = self.vehicle_param.wheel_base
        curvature = math.tan(math.radians(carsteer / 100
                                          * math.degrees(carmax_steer_angle)) / carsteer_ratio) / carwheel_base
        if abs(carspeed) >= speed_epsilon:
            carcurvature_change_rate = (curvature - self.carcurvature) / (
                carspeed * 0.01)
        else:
            carcurvature_change_rate = 0.0
        self.carcurvature = curvature
        cartime = self.localization.header.timestamp_sec
        cargear = self.chassis.gear_location

        if abs(carspeed) >= speed_epsilon:
            if self.startmoving is False:
                self.logger.info(
                    "carspeed !=0 and startmoving is False, Start Recording")
            self.startmoving = True

        if self.startmoving:
            self.cars += carspeed * 0.01
            self.write(
                "%s, %s, %s, %s, %s, %s, %s, %.4f, %s, %s, %s, %s, %s, %s\n" %
                (carx, cary, carz, carspeed, caracceleration, self.carcurvature,
                 carcurvature_change_rate, cartime, cartheta, cargear,
                 self.cars, self.chassis.throttle_percentage,
                 self.chassis.brake_percentage,
                 self.chassis.steering_percentage))
            self.logger.debug(
                "started moving and write data at time %s" % cartime)
        else:
            self.logger.debug("not start moving, do not write data to file")

        self.prev_carspeed = carspeed

    def shutdown(self):
        """
        shutdown node
        """
        self.terminating = True
        self.logger.info("Shutting Down...")
        self.logger.info("File is written into %s" % self.record_file)
        self.file_handler.close()


def main(argv):
    """
    Main node
    """
    node = cyber.Node("rtk_recorder")
    argv = FLAGS(argv)

    log_dir = "/apollo/data/log"
    if len(argv) > 1:
        log_dir = argv[1]

    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    Logger.config(
        log_file=log_dir + "rtk_recorder.log",
        use_stdout=True,
        log_level=logging.DEBUG)
    print("runtime log is in %s%s" % (log_dir, "rtk_recorder.log"))

    record_file = log_dir + "/garage.csv"
    recorder = RtkRecord(record_file)
    atexit.register(recorder.shutdown)
    node.create_reader('/apollo/canbus/chassis',
                       chassis_pb2.Chassis,
                       recorder.chassis_callback)

    node.create_reader('/apollo/localization/pose',
                       localization_pb2.LocalizationEstimate,
                       recorder.localization_callback)

    while not cyber.is_shutdown():
        time.sleep(0.002)


if __name__ == '__main__':
    cyber.init()
    main(sys.argv)
    cyber.shutdown()
