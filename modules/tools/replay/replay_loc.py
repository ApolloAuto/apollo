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
This program can replay a message pb file
"""
import os.path
import sys
import argparse
import glob
import time
import math
from modules.localization.proto import localization_pb2
from modules.transform.proto import transform_pb2
from google.protobuf import text_format
from cyber.python.cyber_py3 import cyber
import modules.tools.common.proto_utils as proto_utils
from cyber.python.cyber_py3 import cyber_time
def topic_publisher(topic, msg, tf_msg,period):
    """publisher"""
    cyber.init()
    node = cyber.Node("replay_loc") 
    sequence_num=0                          
    writer = node.create_writer(topic, localization_pb2.LocalizationEstimate)
    writer_tf = node.create_writer("/tf", transform_pb2.TransformStampeds)
    if period == 0:
        while not cyber.is_shutdown():
            raw_input("Press any key to publish one message...")
            writer.write(msg)
            print("Topic[%s] message published" % topic)
    else:
        print("started to publish topic[%s] message with rate period %s" %
              (topic, period))
        while not cyber.is_shutdown():
            msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
            msg.header.sequence_num = sequence_num
            tf_msg.header.timestamp_sec= msg.header.timestamp_sec
            tf_msg.transforms[0].header.timestamp_sec = msg.header.timestamp_sec
            sequence_num = (sequence_num + 1)%1000000
            writer_tf.write(tf_msg)
            writer.write(msg)
            print(msg)
            time.sleep(period)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="replay a planning result pb file")
    parser.add_argument('pose', nargs='+',type=float)
    args = parser.parse_args()
    pose = args.pose
    if len(pose)!=4 or  len(pose)!=7 :
        print("%d not enough param",len(pose))
    localization=localization_pb2.LocalizationEstimate()
    sequence_num=0
    transforms=transform_pb2.TransformStampeds()
    transfrom=transforms.transforms.add()
    localization.header.timestamp_sec = cyber_time.Time.now().to_sec()
    transfrom.header.timestamp_sec= localization.header.timestamp_sec
    transfrom.header.frame_id = 'world'
    transfrom.child_frame_id = 'localization'
    localization.header.module_name = 'daoyuan'
    localization.header.sequence_num = sequence_num
    sequence_num = sequence_num + 1
    localization.pose.position.x=pose[0]
    localization.pose.position.y=pose[1]
    localization.pose.position.z=0
    transfrom.transform.translation.x=pose[0]
    transfrom.transform.translation.y=pose[1]
    transfrom.transform.translation.z=0
    if len(pose)==4:
        localization.pose.heading=math.atan2((pose[3]-pose[1]),(pose[2]-pose[0]))
        localization.pose.orientation.qx=0
        localization.pose.orientation.qy=0
        localization.pose.orientation.qz=math.sin(localization.pose.heading/2)
        localization.pose.orientation.qw=math.cos(localization.pose.heading/2)
    else:
        localization.pose.orientation.qx=pose[2]
        localization.pose.orientation.qy=pose[3]
        localization.pose.orientation.qz=pose[4]
        localization.pose.orientation.qw=pose[5]
        localization.pose.heading=pose[6]
    transfrom.transform.rotation.qx=0
    transfrom.transform.rotation.qy=0
    transfrom.transform.rotation.qz=localization.pose.orientation.qz
    transfrom.transform.rotation.qw=localization.pose.orientation.qw
    localization.pose.linear_velocity.x=0
    localization.pose.linear_velocity.y=0
    localization.pose.linear_velocity.z=0
    localization.pose.linear_acceleration.x=0
    localization.pose.linear_acceleration.y=0
    localization.pose.linear_acceleration.z=0
    localization.pose.angular_velocity.x=0
    localization.pose.angular_velocity.y=0
    localization.pose.angular_velocity.z=0
    localization.pose.linear_acceleration_vrf.x=0
    localization.pose.linear_acceleration_vrf.y=0
    localization.pose.linear_acceleration_vrf.z=0
    localization.pose.angular_velocity_vrf.x=0
    localization.pose.angular_velocity_vrf.y=0
    localization.pose.angular_velocity_vrf.z=0
    print("heading ",localization.pose.heading)
    topic_publisher("/apollo/localization/pose",localization,transforms,0.01)

