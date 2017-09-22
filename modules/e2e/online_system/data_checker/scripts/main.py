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
import threading
import math
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from car_msgs.msg import ABS, PX2
from novatel_msgs.msg import BESTPOS, INSPVA, Alignment

g_image_msg = Image()
g_can_msg = ABS()
g_rtk_msg = BESTPOS()
g_ins_msg = INSPVA()
g_alig_msg = Alignment()
g_est_msg = PX2()
g_act_msg = PX2()


def est_callback(data):
    global g_est_msg
    g_est_msg = data


def act_callback(data):
    global g_act_msg
    g_act_msg = data


def image_callback(data):
    global g_image_msg
    g_image_msg = data


def rtk_callback(data):
    global g_rtk_msg
    g_rtk_msg = data


def ins_callback(data):
    global g_ins_msg
    g_ins_msg = data


def alig_callback(data):
    global g_alig_msg
    g_alig_msg = data


def can_callback(data):
    global g_can_msg
    g_can_msg = data


def check_msg(time):
    now = rospy.Time.now()
    t = abs((now - time).to_sec())
    rospy.loginfo("diff " + str(t) + " now " + str(now) + " msg " + str(time))
    if t > 2:
        return False
    else:
        return True


def work():
    global g_image_msg
    global g_can_msg
    global g_rtk_msg
    global g_alig_msg
    global g_ins_msg
    global g_est_msg
    global g_act_msg

    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        rospy.loginfo("check image")
        ret = check_msg(g_image_msg.header.stamp)
        if not ret:
            image_status_pub.publish(False)
        else:
            image_status_pub.publish(True)

        rospy.loginfo("check can")
        ret = check_msg(g_can_msg.header.stamp)
        if not ret:
            can_status_pub.publish(False)
        else:
            can_status_pub.publish(True)

        rospy.loginfo("check est")
        ret = check_msg(g_est_msg.header.stamp)
        if not ret:
            est_status_pub.publish(False)
        else:
            est_status_pub.publish(True)

        rospy.loginfo("check act")
        ret = check_msg(g_act_msg.header.stamp)
        if not ret:
            act_status_pub.publish(False)
        else:
            act_status_pub.publish(True)

        rospy.loginfo("check alig")
        ret = check_msg(g_alig_msg.header.stamp)
        if not ret:
            rtk_status_pub.publish(False)
        else:
            if g_rtk_msg.position_type != 48 and g_rtk_msg.position_type != 50 and g_rtk_msg.position_type != 56:
                rtk_status_pub.publish(False)
            else:
                rtk_status_pub.publish(True)

        if g_ins_msg.insstatus != 3:
            ins_status_pub.publish(False)
        else:
            ins_status_pub.publish(True)

        r.sleep()


rospy.init_node('data_check', anonymous=True)

image_sub = rospy.Subscriber(
    '/car_msgs/image',
    Image,
    image_callback,
    queue_size=1,
    buff_size=1024 * 1024 * 16)
can_sub = rospy.Subscriber(
    '/car_msgs/ABS',
    ABS,
    can_callback,
    queue_size=1,
    buff_size=1024 *
    1024 *
    16)
rtk_sub = rospy.Subscriber(
    '/novatel_data/bestpos',
    BESTPOS,
    rtk_callback,
    queue_size=1,
    buff_size=1024 * 1024 * 16)
ins_sub = rospy.Subscriber(
    '/novatel_data/inspva',
    INSPVA,
    ins_callback,
    queue_size=1,
    buff_size=1024 * 1024 * 16)
alig_sub = rospy.Subscriber(
    '/novatel_data/align',
    Alignment,
    alig_callback,
    queue_size=1,
    buff_size=1024 * 1024 * 16)
est_sub = rospy.Subscriber(
    '/car_msgs/estimate_px2',
    PX2,
    est_callback,
    queue_size=1,
    buff_size=1024 * 1024 * 16)
act_sub = rospy.Subscriber(
    '/car_msgs/PX2',
    PX2,
    act_callback,
    queue_size=1,
    buff_size=1024 *
    1024 *
    16)

image_status_pub = rospy.Publisher('/system_info/image', Bool, queue_size=1)
rtk_status_pub = rospy.Publisher('/system_info/rtk', Bool, queue_size=1)
ins_status_pub = rospy.Publisher('/system_info/ins', Bool, queue_size=1)
can_status_pub = rospy.Publisher('/system_info/can', Bool, queue_size=1)
act_status_pub = rospy.Publisher('/system_info/act', Bool, queue_size=1)
est_status_pub = rospy.Publisher('/system_info/est', Bool, queue_size=1)

th = threading.Thread(target=work)
th.setDaemon(True)
th.start()

rospy.spin()
