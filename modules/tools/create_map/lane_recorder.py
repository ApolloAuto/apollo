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
Record localization and mobileye lane detection in CSV format
"""

import argparse
import atexit
import logging
import math
import os
import rospy
import sys

from gflags import FLAGS
from std_msgs.msg import String

from common.logger import Logger
from modules.localization.proto import localization_pb2
from modules.drivers.proto import mobileye_pb2


class LaneRecord(object):
    """
    lane recording class
    """

    def write(self, data):
        """Wrap file write function to flush data to disk"""
        self.file_handler.write(data)
        self.file_handler.flush()

    def __init__(self, record_file):
        self.firstvalid = False
        self.logger = Logger.get_logger("LaneRecord")
        self.record_file = record_file
        self.logger.info("Record file to: " + record_file)

        try:
            self.file_handler = open(record_file, 'w')
        except IOError:
            self.logger.error("Failed to open file %s " % (record_file))
            self.file_handler.close()
            sys.exit(-1)

        self.write("x,y,z,theta,dist_l,conf_l,dist_r,conf_r\n")

        self.localization = localization_pb2.LocalizationEstimate()
        self.mobileye = mobileye_pb2.Mobileye()
        self.mobileye_received = False
        self.terminating = False

    def mobileye_callback(self, data):
        """
        New message received
        """
        if self.terminating is True:
            self.logger.info("terminating when receive mobileye msg")
            return

        self.mobileye.CopyFrom(data)
        self.mobileye_received = True

    def localization_callback(self, data):
        """
        New message received
        """
        if self.terminating is True:
            self.logger.info("terminating when receive localization msg")
            return

        if not self.mobileye_received:
            self.logger.info(
                "mobileye not received when localization is received")
            return

        self.localization.CopyFrom(data)
        carx = self.localization.pose.position.x
        cary = self.localization.pose.position.y
        carz = self.localization.pose.position.z
        cartheta = self.localization.pose.heading
        dist_l = self.mobileye.aftermarket_669.distance_to_lane_l
        conf_l = self.mobileye.aftermarket_669.lane_conf_left
        dist_r = self.mobileye.aftermarket_669.distance_to_lane_r
        conf_r = self.mobileye.aftermarket_669.lane_conf_right

        self.write(
            "%s, %s, %s, %s, %s, %s, %s, %s\n" %
            (carx, cary, carz, cartheta, dist_l, conf_l, dist_r, conf_r))

    def shutdown(self):
        """
        shutdown rosnode
        """
        self.terminating = True
        self.logger.info("Shutting Down...")
        self.logger.info("file is written into %s" % self.record_file)
        self.file_handler.close()
        rospy.sleep(0.1)


def main(argv):
    """
    Main rosnode
    """
    rospy.init_node('lane_recorder', anonymous=True)

    parser = argparse.ArgumentParser(
        description='Record Localization and Mobileye Lane Detection in CSV Format')
    parser.add_argument(
        '-d',
        '--dir',
        help='Output and log directory',
        type=str,
        default='/tmp/')
    parser.add_argument(
        '-o',
        '--output_file',
        help='Output CSV file name',
        type=str,
        default='lane.csv')
    args = vars(parser.parse_args())

    log_dir = args['dir']
    record_file = log_dir + "/" + args['output_file']

    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    Logger.config(
        log_file=log_dir + "lane_recorder.log",
        use_stdout=True,
        log_level=logging.DEBUG)
    print("runtime log is in %s%s" % (log_dir, "lane_recorder.log"))
    recorder = LaneRecord(record_file)
    atexit.register(recorder.shutdown)
    rospy.Subscriber('/apollo/sensor/mobileye', mobileye_pb2.Mobileye,
                     recorder.mobileye_callback)

    rospy.Subscriber('/apollo/localization/pose',
                     localization_pb2.LocalizationEstimate,
                     recorder.localization_callback)

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
