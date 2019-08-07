#!/usr/bin/env python

import argparse
import atexit
import logging
import os
import sys

import rospy
import scipy.signal as signal
from numpy import genfromtxt

from modules.drivers.gnss.proto import ins_pb2


class InsStat(object):
    def __init__(self):
        self.insstat_pub = rospy.Publisher('/apollo/sensor/gnss/ins_stat', ins_pb2.InsStat, queue_size=1)
        self.sequence_num = 0
        self.terminating = False

    def publish_statmsg(self):
        insstat = ins_pb2.InsStat()
        now = rospy.get_time()
        insstat.header.timestamp_sec = now
        insstat.ins_status = 3
        insstat.pos_type = 56
        rospy.loginfo("%s"%insstat)
        self.insstat_pub.publish(insstat)

    def shutdown(self):
        """
        shutdown rosnode
        """
        self.terminating = True
        self.logger.info("Shutting Down...")
        rospy.sleep(0.2)

def main():
    """
    Main rosnode
    """
    rospy.init_node('ins_stat_publisher', anonymous=True)
    ins_stat = InsStat()
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        ins_stat.publish_statmsg()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
