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
This module publishes the car control message.

"""

import rospy
import time
import threading
from car_msgs.msg import ABS
from car_msgs.msg import ABM
from car_msgs.msg import BCM
from car_msgs.msg import ECM
from car_msgs.msg import SAS
from car_msgs.msg import PX2
from rc_msgs.msg import RCCommand
from novatel_msgs.msg import INSPVA
from std_msgs.msg import Bool
import sys
import signal
import numpy as np


class CarAct(object):
    """Summary of class here.

    Subscribe the CAN message and generate the car control message.

    Attributes:
        current_mode: current car control mode.
        abs_lock: threading lock for the abs CAN message.
        abm_lock: threading lock for the abm CAN message.
        ecm_lock: threading lock for the ecm CAN message.
        bcm_lock: threading lock for the bcm CAN message.
        sas_lock: threading lock for the sas CAN message.
        px2_lock: threading lock for the px2 CAN message.
        rc_lock: threading lock for the rc CAN message.
        inspva_lock: threading lock for the inspva message.
	abs_message: abs CAN message.
	abm_message: abm CAN message.
	bcm_message: bcm CAN message.
	ecm_message: ecm CAN message.
	sas_message: sas CAN message.
	rc_message: rc message.
	px2_message: the computed speed and steering_angle from the PX2.
 	inspva_message: inspva message from the novatel.
 	mode: car control mode.
	mode_sub: subscriber for the mode message.
	abs_sub: subscriber for the abs CAN message.
	abm_sub: subscriber for the abm CAN message.	
	ecm_sub: subscriber for the ecm CAN message.	
	sas_sub: subscriber for the sas CAN message.	
	bcm_sub: subscriber for the bcm CAN message.	
	px2_sub: subscriber for the computed speed and steering_angle from the PX2.	
	rc_sub: subscriber for the rc message.	
	inspva_sub: subscriber for the novatel inspva message.
	px2_pub: publisher for the car control message.	
    """
    def __init__(self):
        """ Inits the CarAct ."""
        self.current_mode = 0
        self.abs_lock = threading.Lock()
        self.abm_lock = threading.Lock()
        self.ecm_lock = threading.Lock()
        self.bcm_lock = threading.Lock()
        self.sas_lock = threading.Lock()
        self.px2_lock = threading.Lock()
        self.rc_lock = threading.Lock()
        self.inspva_lock = threading.Lock()
        self.abs_message = ABS()
        self.abm_message = ABM()
        self.bcm_message = BCM()
        self.ecm_message = ECM()
        self.sas_message = SAS()
        self.rc_message = RCCommand()
        self.px2_message = PX2()
        self.inspva_message = INSPVA()
        self.mode = Bool("True")
        self.mode_sub = rospy.Subscriber(
            '/temp/mode', Bool, self.mode_callback, queue_size=1)
        self.abs_sub = rospy.Subscriber(
            '/car_msgs/ABS', ABS, self.abs_callback, queue_size=1)
        self.abm_sub = rospy.Subscriber(
            '/car_msgs/ABM', ABM, self.abm_callback, queue_size=1)
        self.ecm_sub = rospy.Subscriber(
            '/car_msgs/ECM', ECM, self.ecm_callback, queue_size=1)
        self.sas_sub = rospy.Subscriber(
            '/car_msgs/SAS', SAS, self.sas_callback, queue_size=1)
        self.bcm_sub = rospy.Subscriber(
            '/car_msgs/BCM', BCM, self.bcm_callback, queue_size=1)
        self.px2_sub = rospy.Subscriber(
            '/car_msgs/estimate_px2',
            PX2,
            self.px2_callback,
            queue_size=1)
        self.rc_sub = rospy.Subscriber(
            '/car_msgs/rc_command',
            RCCommand,
            self.rc_callback,
            queue_size=10)
        self.inspva_sub = rospy.Subscriber(
            '/novatel_data/inspva',
            INSPVA,
            self.inspva_callback,
            queue_size=1)
        self.px2_pub = rospy.Publisher('/car_msgs/PX2', PX2, queue_size=1)

    def inspva_callback(self, data):
        """inspva_callback function."""
        self.inspva_lock.acquire()
        self.inspva_message = data
        self.inspva_lock.release()

    def mode_callback(self, data):
        """mode_callback function."""
        self.mode = data

    def abs_callback(self, data):
        """abs_callback function."""
        self.abs_lock.acquire()
        self.abs_message = data
        self.abs_lock.release()

    def abm_callback(self, data):
        """abm_callback function."""
        self.abm_lock.acquire()
        self.abm_message = data
        self.abm_lock.release()

    def ecm_callback(self, data):
        """ecm_callback function."""
        self.ecm_lock.acquire()
        self.ecm_message = data
        self.ecm_lock.release()

    def sas_callback(self, data):
        """sas_callback function."""
        self.sas_lock.acquire()
        self.sas_message = data
        self.sas_lock.release()

    def bcm_callback(self, data):
        """bcm_callback function."""
        self.bcm_lock.acquire()
        self.bcm_message = data
        self.bcm_lock.release()

    def px2_callback(self, data):
        """px2_callback function."""
        self.px2_lock.acquire()
        self.px2_message = data
        self.px2_lock.release()

    def rc_callback(self, data):
        """rc_callback function."""
        self.rc_lock.acquire()
        self.rc_message = data
        self.rc_lock.release()

    def roll(self):
        """ros roll main function."""
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.rc_lock.acquire()
            rc = self.rc_message
            self.rc_lock.release()

            px2_message = PX2()
            px2_message.Mode = 2
            px2_message.header.stamp = rospy.Time.now()

            if rc.enable:
                px2_message.TarSteeringAngle = rc.steering_angle

                # Mock rc angle
                self.px2_lock.acquire()
                px2_message = self.px2_message
                self.px2_lock.release()

                px2_message.TarAcce = rc.acceleration
                px2_message.Mode = 2
                if px2_message.TarAcce < 0:
                    px2_message.DecToStop = 0
                else:
                    px2_message.DecToStop = 1

            else:
                self.px2_lock.acquire()
                target_angle = self.px2_message.TarSteeringAngle
                target_speed = self.px2_message.TarAcce
                self.px2_lock.release()

                self.ecm_lock.acquire()
                current_speed = self.ecm_message.VehicleSpd
                self.ecm_lock.release()

                self.sas_lock.acquire()
                angle = self.sas_message.SteeringWheelAngle
                angle_sign = self.sas_message.SteeringWheelAngleSign
                if angle_sign == 1:
                    angle = -angle
                self.sas_lock.release()

                self.inspva_lock.acquire()
                longitude = self.inspva_message.longitude
                latitude = self.inspva_message.latitude
                heading = self.inspva_message.azimuth
                self.inspva_lock.release()

                px2_message.TarAcce = target_acce  
                px2_message.TarSteeringAngle = target_angle

            self.px2_pub.publish(px2_message)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('car_act', anonymous=True)
    car_act = CarAct()
    car_act.roll()
    rospy.spin()
