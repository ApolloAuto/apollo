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
This module receives the image and output the car control message .

"""
import rospy
import cv2
import sys
from models.nv_cls import NvCls
from models.nv_reg import NvReg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from car_msgs.msg import PX2
import keras
from keras import backend as K
import tensorflow as tf
import time


class CarControllEstimate(object):
    """Summary of class here.

    Subscribe the image message and generate the car control message.

    Attributes:
        px2_pub: publish the car control message.
        bridge: opencv bridge.
        _loaded: weights load flag.
        _M: prediction module.\
        image_sub: image message subscriber.
    """

    def __init__(self):
        """ Inits the CarControllEstimate ."""
        self.px2_pub = rospy.Publisher(
            "/car_msgs/estimate_px2", PX2, queue_size=1)
        self.bridge = CvBridge()
        self._loaded = False
        self._M = NvCls()
        self._M.load_weights(
            "/workspace/roadhacker/outputs/ces/h7_cls_chp.h5.0605b")
        self._loaded = True
        print "param loaded"
        self.image_sub = rospy.Subscriber(
            '/car_msgs/image',
            Image,
            self.callback,
            queue_size=1,
            buff_size=1024 * 1024 * 8)

    def callback(self, data):
        """ Image callback . Get the image and output the car control_estimate ."""
        try:
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(data, "rgb8")
            rospy.loginfo("received image")
            # call method
            img = cv2.resize(cv_image, (320, 320))
            if self._loaded:
                t = time.time()
                angle_pred, speed_pred = self._M.predict(img)
                angle_pred = angle_pred * -1.
                speed_pred = speed_pred  
            else:
                print "not loaded"
            px2_msg = PX2()
            px2_msg.header.stamp = rospy.Time.now()
            px2_msg.TarAcce = speed_pred  
            px2_msg.TarSteeringAngle = angle_pred  
            px2_msg.Curvature = 0  
            px2_msg.Mode = 2
            px2_msg.PX2_UID = 0

            self.px2_pub.publish(px2_msg)
        except CvBridgeError as e:
            rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node('control_estimate', anonymous=True)
    ce = CarControllEstimate()

    rospy.spin()
