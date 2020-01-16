#!/usr/bin/env python

import argparse
import atexit
import logging
import os
import sys
import time


#from common.logger import Logger
from cyber_py import cyber
from cyber_py import cyber_time

from modules.drivers.gnss.proto import ins_pb2


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
