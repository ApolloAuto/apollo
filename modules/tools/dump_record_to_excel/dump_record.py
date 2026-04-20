#!/usr/bin/env python3

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
This program can dump a cyber record into separate text files that contains
the pb messages
"""
import os
os.system("pip install pandas")
os.system("pip install openpyxl")
import argparse
import sys
import yaml
import pandas as pd
from openpyxl import load_workbook
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import record
from modules.tools.common.message_manager import PbMessageManager

g_message_manager = PbMessageManager()


def parse_attrib(attrib_name, topic, keyword_list):
    if ('Repeated' in str(type(eval(attrib_name)))): # Dont support repeated component
        return
    if (isinstance(eval(attrib_name), str) or isinstance(eval(attrib_name), int) or isinstance(eval(attrib_name), float)):
        if not keyword_list or (keyword_list and [True for i in keyword_list if i in attrib_name]):
            attrib_list[topic][attrib_name] = []
    else:
        for j in [(attrib_name + '.' + i) for i in dir(eval(attrib_name)) if ord('a') <= ord(i[0]) <= ord('z') and i != "yield"]:
            parse_attrib(j, topic, keyword_list)


def dump_record(in_record, out_dir, start_time, duration, topic_list):
    global msg
    global attrib_list
    Flag_initial = True

    topic_name_list = [i["name"] for i in topic_list if i["name"]]
    attrib_list = {i:{} for i in topic_name_list}
    timestamp_list = {i:[] for i in topic_name_list}
    for i, topic in enumerate(topic_name_list):
        meta_msg = g_message_manager.get_msg_meta_by_topic(topic)
        if meta_msg is None:
            print('Unknown topic: %s' % topic)
            continue
        msg = meta_msg.msg_type()
        parse_attrib("msg", topic, topic_list[i]["keyword"])

    # print(topic_list)
    # print(topic_name_list)
    # print(attrib_list)

    try:
        freader = record.RecordReader(in_record)
    except Exception:
        print('Cannot open record file %s' % in_record)
    else:
        for BagMessage in freader.read_messages():
            if Flag_initial:
                initial_time = BagMessage.timestamp
                Flag_initial = False

            t_sec = BagMessage.timestamp
            if start_time and t_sec / 1e9 < start_time:
                # print('Not yet reached the start time')
                continue
            if duration:
                if start_time:
                    if t_sec / 1e9 >= start_time + duration:
                        print('Done')
                        break
                elif t_sec / 1e9 >= initial_time / 1e9 + duration:
                    print('Done')
                    break

            topic = BagMessage.topic
            if topic == '/apollo/sensor/mobileye':
                continue
            if topic in topic_name_list:
                meta_msg = g_message_manager.get_msg_meta_by_topic(topic)
                if meta_msg is None:
                    print('Unknown topic: %s' % topic)
                    continue
                msg = meta_msg.msg_type()
                msg.ParseFromString(BagMessage.message)
                for key, value in attrib_list[topic].items():
                    value.append(eval(key))
                timestamp_list[topic].append(msg.header.timestamp_sec)

        writer = pd.ExcelWriter(out_dir + '/dump_record_result.xlsx')
        for i, topic in enumerate(topic_name_list):
            Raw_Data = pd.DataFrame(attrib_list[topic], timestamp_list[topic], attrib_list[topic].keys())
            Raw_Data.to_excel(writer, sheet_name=topic.replace('/','_'))
        writer.save()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="A tool to dump the protobuf messages in a cyber record into text files"
    )
    parser.add_argument(
        "in_record",
        action="store",
        type=str,
        help="the input cyber record")
    parser.add_argument(
        "-s",
        "--start_time",
        action="store",
        type=float,
        help="start time to parse record, eg: 1635927111.6681252")
    parser.add_argument(
        "-d",
        "--duration",
        action="store",
        type=float,
        help="duration of parse record, unit:s")
    parser.add_argument(
        "out_dir",
        action="store",
        type=str,
        help="the output directory for the dumped file")

    args = parser.parse_args()
    if not os.path.exists(args.out_dir):
        print('%s does not exist' % args.out_dir)
        sys.exit(1)

    yaml_file = '/apollo/modules/tools/dump_record/dump_record.yaml'
    with open(yaml_file, 'r') as f:
        params = yaml.safe_load(f)
    topic_list = params['Topic']
    # print(topic_list)

    dump_record(args.in_record, args.out_dir, args.start_time, args.duration, topic_list)

    # print(attrib_list)