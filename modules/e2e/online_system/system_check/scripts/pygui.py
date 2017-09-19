#! /usr/bin/env python

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

#from Tkinter import *
from Tkinter import TK
from Tkinter import Frame
from std_msgs.msg import String
import time
import threading
import rospy

g_mount_status = False
g_image_status = False
g_can_status = False
g_ins_status = False
g_rtk_status = False
g_est_status = False
g_act_status = False


def est_callback(data):
    """car_control_estimate_callback function."""
    global g_est_status
    g_est_status = data.data
    rospy.loginfo("received est" + str(g_est_status))


def act_callback(data):
    """car_act_callback function."""
    global g_act_status
    g_act_status = data.data
    rospy.loginfo("received act " + str(g_act_status))


def ins_callback(data):
    """inspva_callback function."""
    global g_ins_status
    g_ins_status = data.data
    rospy.loginfo("received ins " + str(g_ins_status))


def rtk_callback(data):
    """rtk_status_callback function."""
    global g_rtk_status
    g_rtk_status = data.data
    rospy.loginfo("received rtk" + str(g_rtk_status))


def mount_callback(data):
    """disk_mount_status_callback function."""
    global g_mount_status
    g_mount_status = data.data
    rospy.loginfo("received mount " + str(g_mount_status))


def image_callback(data):
    """image_receive_status_callback function."""
    global g_image_status
    g_image_status = data.data
    rospy.loginfo("received image " + str(g_image_status))


def can_callback(data):
    """can_status_callback function."""
    global g_can_status
    g_can_status = data.data
    rospy.loginfo("received can " + str(g_can_status))


def work():
    """A monitor is initialized to monitor all import status."""
    global g_mount_status
    global g_image_status
    global g_can_status
    global g_rtk_status
    global g_ins_status
    global g_est_status
    global g_act_status

    is_show = rospy.get_param("~is_show", False)

    rospy.loginfo("~is show " + str(is_show))

    try:
        while True:
            if (g_rtk_status and g_can_status and g_image_status
                and (is_show or g_mount_status) and (not is_show
                                                     or (g_est_status 
						     and g_act_status 
						     and g_ins_status))):
                frame['bg'] = '#00FF00'
                rospy.loginfo("all true")
            else:
                frame['bg'] = '#FF0000'
                rospy.loginfo("not all true")
                if is_show:
                    rospy.loginfo(
                        "image " +
                        str(g_image_status) +
                        " can " +
                        str(g_can_status) +
                        " rtk " +
                        str(g_rtk_status) +
                        " est " +
                        str(g_est_status) +
                        " act " +
                        str(g_act_status) +
                        " ins " +
                        str(g_ins_status))
                else:
                    rospy.loginfo(
                        "image " +
                        str(g_image_status) +
                        " can " +
                        str(g_can_status) +
                        " rtk " +
                        str(g_rtk_status) +
                        " mount " +
                        str(g_mount_status))
            time.sleep(1)
    except KeyboardInterrupt:
        exit(0)


rospy.init_node('system_check', anonymous=True)
mount_sub = rospy.Subscriber(
    '/system_info/mount',
    Bool,
    mount_callback,
    queue_size=1,
    buff_size=1024 * 1024 * 8)
can_sub = rospy.Subscriber(
    '/system_info/can',
    Bool,
    can_callback,
    queue_size=1,
    buff_size=1024 * 1024 * 8)
image_sub = rospy.Subscriber(
    '/system_info/image',
    Bool,
    image_callback,
    queue_size=1,
    buff_size=1024 * 1024 * 8)
ins_sub = rospy.Subscriber(
    '/system_info/ins',
    Bool,
    ins_callback,
    queue_size=1,
    buff_size=1024 * 1024 * 8)
rtk_sub = rospy.Subscriber(
    '/system_info/rtk',
    Bool,
    rtk_callback,
    queue_size=1,
    buff_size=1024 * 1024 * 8)
est_sub = rospy.Subscriber(
    '/system_info/est',
    Bool,
    est_callback,
    queue_size=1,
    buff_size=1024 * 1024 * 8)
act_sub = rospy.Subscriber(
    '/system_info/act',
    Bool,
    act_callback,
    queue_size=1,
    buff_size=1024 * 1024 * 8)

root = Tk(className='system_check')

frame = Frame(root, width=1000, height=1000)
frame.pack()

th = threading.Thread(target=work)
th.setDaemon(True)
th.start()

root.mainloop()
