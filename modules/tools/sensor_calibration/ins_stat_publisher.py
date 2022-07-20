#!/usr/bin/env python3

###############################################################################
# Copyright 2020 The Apollo Authors. All Rights Reserved.
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
import atexit
import logging
import os
import sys
import time

#from common.logger import Logger
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time

from modules.common_msgs.sensor_msgs import ins_pb2


class InsStat(object):
    def __init__(self,node):
        self.insstat_pub = node.create_writer('/apollo/sensor/gnss/ins_stat', ins_pb2.InsStat)
        self.sequence_num = 0
        self.terminating = False

    def publish_statmsg(self):
        insstat = ins_pb2.InsStat()
        now = cyber_time.Time.now().to_sec()
        insstat.header.timestamp_sec = now
        insstat.header.module_name = "ins_stat"
        insstat.header.sequence_num = self.sequence_num
        self.sequence_num = self.sequence_num + 1
        insstat.ins_status = 3
        insstat.pos_type = 56
        #self.logger.info("%s"%insstat)
        self.insstat_pub.write(insstat)

    def shutdown(self):
        """
        shutdown rosnode
        """
        self.terminating = True
        #self.logger.info("Shutting Down...")
        time.sleep(0.2)

def main():
    """
    Main rosnode
    """
    node=cyber.Node('ins_stat_publisher')
    ins_stat = InsStat(node)
    while not cyber.is_shutdown():
        now = cyber_time.Time.now().to_sec()
        ins_stat.publish_statmsg()
        sleep_time = 0.5 - (cyber_time.Time.now().to_sec() - now)
        if sleep_time > 0:
            time.sleep(sleep_time)

if __name__ == '__main__':
    cyber.init()
    main()
    cyber.shutdown()
