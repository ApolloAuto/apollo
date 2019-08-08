#!/usr/bin/env python

import argparse
import atexit
import logging
import os
import sys

import rospy
from gflags import FLAGS

from modules.localization.proto import localization_pb2
from modules.localization.proto import gps_pb2

class OdomPublisher(object):
    def __init__(self):
        self.localization = localization_pb2.LocalizationEstimate()
        self.gps_odom_pub = rospy.Publisher('/apollo/sensor/gnss/odometry', gps_pb2.Gps, queue_size=1) 
        self.sequence_num = 0
        self.terminating = False
        self.position_x = 0
        self.position_y = 0
        self.position_z = 0
        self.orientation_x = 0
        self.orientation_y = 0
        self.orientation_z = 0
        self.orientation_w = 0
        self.linear_velocity_x = 0 
        self.linear_velocity_y = 0
        self.linear_velocity_z = 0

    def localization_callback(self, data):
        """
        New message received
        """
        self.localization.CopyFrom(data)
        self.position_x = self.localization.pose.position.x
        self.position_y = self.localization.pose.position.y
        self.position_z = self.localization.pose.position.z
        self.orientation_x = self.localization.pose.orientation.qx
        self.orientation_y = self.localization.pose.orientation.qy
        self.orientation_z = self.localization.pose.orientation.qz
        self.orientation_w = self.localization.pose.orientation.qw
        self.linear_velocity_x = self.localization.pose.linear_velocity.x
        self.linear_velocity_y = self.localization.pose.linear_velocity.y
        self.linear_velocity_z = self.localization.pose.linear_velocity.z

    def odom_callback(self):
        odom = gps_pb2.Gps()
        now = rospy.get_time()
        odom.header.timestamp_sec = now
        odom.localization.position.x = self.position_x
        odom.localization.position.y = self.position_y
        odom.localization.position.z = self.position_z
        odom.localization.orientation.qx = self.orientation_x
        odom.localization.orientation.qy = self.orientation_y
        odom.localization.orientation.qz = self.orientation_z
        odom.localization.orientation.qw = self.orientation_w
        odom.localization.linear_velocity.x = self.linear_velocity_x
        odom.localization.linear_velocity.y = self.linear_velocity_y
        odom.localization.linear_velocity.z = self.linear_velocity_z
        rospy.loginfo("%s"%odom)
        self.gps_odom_pub.publish(odom)

    def shutdown(self):
        """
        shutdown rosnode
        """
        self.terminating = True
        self.logger.info("Shutting Down...")
        self.file_handler.close()
        rospy.sleep(0.1)

def main():
    """
    Main rosnode
    """
    rospy.init_node('odom_publisher', anonymous=True)
    odom = OdomPublisher()
    rospy.Subscriber('/apollo/localization/pose', localization_pb2.LocalizationEstimate, odom.localization_callback)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        odom.odom_callback()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
