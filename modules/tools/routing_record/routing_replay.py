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
import os
import sys
import argparse
import glob
import time
import math
from modules.localization.proto import localization_pb2
from modules.map.proto.map_speed_bump_pb2 import SpeedBump
from modules.routing.proto.routing_pb2 import RoutingRequest, RoutingResponse
from modules.transform.proto import transform_pb2
from modules.canbus.proto import chassis_pb2
from google.protobuf import text_format
from cyber.python.cyber_py3 import cyber
import modules.tools.common.proto_utils as proto_utils
from cyber.python.cyber_py3 import cyber_time
localization_=localization_pb2.LocalizationEstimate()
canbus_=chassis_pb2.Chassis()
speed_=float(0)
localizationx_=float(0)
localizationy_=float(0)
get_routing_=False
def topic_publisher(topic, msg, tf_msg,period):
    """publisher"""
    
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
def callback_localization(data):
    global localizationx_
    localizationx_=data.pose.position.x
    global localizationy_
    localizationy_=data.pose.position.y
def callback_canbus(data):
    global speed_
    speed_=data.speed_mps
def callback_routing(data):
    global get_routing_
    get_routing_=True
if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="input routing file")
    parser.add_argument(
        "path", action="store", type=str, help="routing path")
    args = parser.parse_args()
    fpath=args.path
    files=os.listdir(fpath)
    flen=len(files)
    routing_list=[RoutingRequest()]*flen
    for file in files:
        file_name=file.replace(".txt","")
        name_list=file_name.split("_")
        with open(os.path.join(fpath,file),'r') as  fread:
            routing_data=fread.read()
            rq_data=RoutingRequest()
            text_format.Parse(routing_data, rq_data)
            routing_list[int(name_list[1])]=rq_data
        fread.close()
    cyber.init()
    node = cyber.Node("replay_routing") 
    localizationsub = node.create_reader( '/apollo/localization/pose', localization_pb2.LocalizationEstimate,
            callback_localization)
    canbussub=node.create_reader('/apollo/canbus/chassis',chassis_pb2.Chassis,
            callback_canbus)
    routingpub=node.create_writer("/apollo/routing_request",RoutingRequest)
    routing_response_sub=node.create_reader('/apollo/routing_response', RoutingResponse,
            callback_routing)
    arrival=0
    now_num=0
    update=1
    routingrq=RoutingRequest()
    print("Drive DKIT to start point and press Enter to start")
    kk=input()
    print("flen",flen)
    while not cyber.is_shutdown() and now_num<flen:
        dis=(localizationx_-routing_list[now_num].waypoint[0].pose.x)**2+ \
            (localizationy_-routing_list[now_num].waypoint[0].pose.y)**2
        if dis<49 and speed_<0.2 and now_num!=flen:
            arrival=1
            print("arrive",now_num+1)
            print(routing_list[now_num].waypoint[0])
            time.sleep(5)
            routingrq=routing_list[now_num]
            routingrq.waypoint[0].pose.x=localizationx_
            routingrq.waypoint[0].pose.y=localizationy_
            routingrq.waypoint[0].ClearField("id")
            routingrq.waypoint[0].ClearField("s")
            routingrq.header.timestamp_sec=cyber_time.Time.now().to_sec()
            routingpub.write(routingrq)
            print("routing_rq",now_num)
            now_num=now_num+1
        time.sleep(0.01)
    
            




