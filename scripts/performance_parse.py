#!/usr/bin/env python3

###############################################################################
# Copyright 2024 The Apollo Authors. All Rights Reserved.
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
performance_parse.py: parse performance results dumped by cyber_performance

Usage：
    1. parse the performance results
       > python3 performance_parse.py -f performance_dumps.06-20-2024.json
"""

import argparse
import subprocess
import os
import re
import sys
import json
import numpy as np
import requests
from datetime import datetime

PERFORMANCE_FILE = None
FORMAT_STR = "%m/%d/%Y, %H:%M:%S"

PERFORMANCE_ALL = {}
PERFORMANCE_AUTODRIVE = {}

def parse_result():
    """
    parse result read from dumped json file of cyber_performance
    """
    global PERFORMANCE_FILE
    global PERFORMANCE_ALL
    global PERFORMANCE_AUTODRIVE
    raw_result = []

    if not os.path.exists(PERFORMANCE_FILE):
        print(f"[ERROR] {PERFORMANCE_FILE} not exists!")
        exit(1)
    with open(PERFORMANCE_FILE, "r") as f:
        raw_result = f.readlines()

    for raw in raw_result:
        sample = json.loads(raw)
        for process in sample["data"]:
            if process not in PERFORMANCE_ALL:
                PERFORMANCE_ALL[process] = {
                    "BASIC": {
                        "cpu_usage": [],
                        "gpu_usage": [],
                        "memory": [],
                        "gpu_memory": [],
                    },
                    "BLOCK_DEVICE_IO": {
                        "io_wait_usage": [],
                        "block_device_io_read": [],
                        "block_device_io_write": [],
                        "devices_io_raw": {},
                    },
                    "ETHERNET_DEVICE_IO": {
                        "ethernet_device_io_read": [],
                        "ethernet_device_io_write": [],
                        "devices_io_raw": {},
                    },
                    "E2E_LATENCY": {},
                    "time": []
                }
            if process not in PERFORMANCE_AUTODRIVE:
                PERFORMANCE_AUTODRIVE[process] = {
                    "BASIC": {
                        "cpu_usage": [],
                        "gpu_usage": [],
                        "memory": [],
                        "gpu_memory": [],
                    },
                    "BLOCK_DEVICE_IO": {
                        "io_wait_usage": [],
                        "block_device_io_read": [],
                        "block_device_io_write": [],
                        "devices_io_raw": {},
                    },
                    "ETHERNET_DEVICE_IO": {
                        "ethernet_device_io_read": [],
                        "ethernet_device_io_write": [],
                        "devices_io_raw": {},
                    },
                    "E2E_LATENCY": {},
                    "time": []
                }
            autodrive_status = sample["data"][process]["autodrive"]
            for i in range(len(autodrive_status)):
                if autodrive_status[i]:
                    for monitor_ele in sample["data"][process]:
                        if monitor_ele == "autodrive":
                            continue
                        if monitor_ele.startswith("time"):
                            PERFORMANCE_AUTODRIVE[process]["time"].append(
                                sample["data"][process][monitor_ele][i])
                            continue
                        
                        monitor_ele_list = monitor_ele.split(" - ")
                        metric_type = monitor_ele_list[0]
                        metric_device = None
                        metric_value = None
                        if len(monitor_ele_list) > 2:
                            metric_device = monitor_ele_list[1]
                            metric_name = re.sub(r'\(.*?\)', '', monitor_ele_list[2])
                        else:
                            metric_name = re.sub(r'\(.*?\)', '', monitor_ele_list[1])
                        if metric_type not in PERFORMANCE_AUTODRIVE[process]:
                            continue
                        if metric_type == "BASIC":
                            PERFORMANCE_AUTODRIVE[process][metric_type][metric_name].append(
                                float(sample["data"][process][monitor_ele][i]))
                        elif metric_type == "E2E_LATENCY":
                            try:
                                if metric_name not in PERFORMANCE_AUTODRIVE[process][metric_type]:
                                    PERFORMANCE_AUTODRIVE[process][metric_type][metric_name] = []
                                PERFORMANCE_AUTODRIVE[process][metric_type][metric_name].append(
                                    float(sample["data"][process][monitor_ele][i]))
                            except:
                                continue
                        else:
                            if metric_device is None or metric_device == "system":
                                PERFORMANCE_AUTODRIVE[process][metric_type][metric_name].append(
                                    float(sample["data"][process][monitor_ele][i]))
                            else:
                                if metric_device not in \
                                        PERFORMANCE_AUTODRIVE[process][metric_type]["devices_io_raw"]:
                                    PERFORMANCE_AUTODRIVE[process][metric_type]["devices_io_raw"][metric_device] = {}
                                if metric_name not in PERFORMANCE_AUTODRIVE[
                                        process][metric_type]["devices_io_raw"][metric_device]:
                                    PERFORMANCE_AUTODRIVE[process][
                                        metric_type]["devices_io_raw"][metric_device][metric_name] = []
                                PERFORMANCE_AUTODRIVE[process][metric_type]["devices_io_raw"][
                                    metric_device][metric_name].append(float(sample["data"][process][monitor_ele][i]))
                        
                for monitor_ele in sample["data"][process]:
                    if monitor_ele == "autodrive":
                        continue
                    if monitor_ele.startswith("time"):
                        PERFORMANCE_ALL[process]["time"].append(
                            sample["data"][process][monitor_ele][i])
                        continue
                    
                    monitor_ele_list = monitor_ele.split(" - ")
                    metric_type = monitor_ele_list[0]
                    metric_device = None
                    metric_value = None
                    if len(monitor_ele_list) > 2:
                        metric_device = monitor_ele_list[1]
                        metric_name = re.sub(r'\(.*?\)', '', monitor_ele_list[2])
                    else:
                        metric_name = re.sub(r'\(.*?\)', '', monitor_ele_list[1])
                    if metric_type not in PERFORMANCE_ALL[process]:
                        continue
                    if metric_type == "BASIC":
                        PERFORMANCE_ALL[process][metric_type][metric_name].append(
                            float(sample["data"][process][monitor_ele][i]))
                    elif metric_type == "E2E_LATENCY":
                        try:
                            if metric_name not in PERFORMANCE_ALL[process][metric_type]:
                                PERFORMANCE_ALL[process][metric_type][metric_name] = []
                            PERFORMANCE_ALL[process][metric_type][metric_name].append(
                                float(sample["data"][process][monitor_ele][i]))
                        except:
                            continue
                    else:
                        if metric_device is None or metric_device == "system":
                            PERFORMANCE_ALL[process][metric_type][metric_name].append(
                                float(sample["data"][process][monitor_ele][i]))
                        else:
                            if metric_device not in \
                                    PERFORMANCE_ALL[process][metric_type]["devices_io_raw"]:
                                PERFORMANCE_ALL[process][metric_type]["devices_io_raw"][metric_device] = {}
                            if metric_name not in PERFORMANCE_ALL[
                                    process][metric_type]["devices_io_raw"][metric_device]:
                                PERFORMANCE_ALL[process][metric_type]["devices_io_raw"][metric_device][metric_name] = []
                            PERFORMANCE_ALL[process][metric_type]["devices_io_raw"][metric_device][metric_name].append(
                                float(sample["data"][process][monitor_ele][i]))
    tab = "  "
    print("Performance: ")
    for p in PERFORMANCE_ALL:
        print(f"{tab}{p}:")
        for ele in PERFORMANCE_ALL[p]:
            if ele == "time":
                continue
            print(f"{tab}{tab}{ele}:")
            for mertic in PERFORMANCE_ALL[p][ele]:
                if mertic == "devices_io_raw":
                    for dev in PERFORMANCE_ALL[p][ele][mertic]:
                        print(f"{tab}{tab}{tab}{dev}:")
                        for dev_metric in PERFORMANCE_ALL[p][ele][mertic][dev]:
                            avg = round(
                                np.mean(PERFORMANCE_ALL[p][ele][mertic][dev][dev_metric]), 2)
                            percentile_90 = round(
                                np.percentile(PERFORMANCE_ALL[p][ele][mertic][dev][dev_metric], 90), 2)
                            print(f"{tab}{tab}{tab}{tab}{dev_metric}: \t[avg: {avg}]\t[90th: {percentile_90}]")
                else:
                    if len(PERFORMANCE_ALL[p][ele][mertic]) == 0:
                        continue
                    avg = round(np.mean(PERFORMANCE_ALL[p][ele][mertic]), 2)
                    percentile_90 = round(np.percentile(PERFORMANCE_ALL[p][ele][mertic], 90), 2)
                    print(f"{tab}{tab}{tab}{mertic}: \t[avg: {avg}]\t[90th: {percentile_90}]")
            
    print("\n")
    print("Performance during autodrive: ")
    for p in PERFORMANCE_AUTODRIVE:
        print(f"{tab}{p}:")
        for ele in PERFORMANCE_AUTODRIVE[p]:
            if ele == "time":
                continue
            print(f"{tab}{tab}{ele}:")
            for mertic in PERFORMANCE_AUTODRIVE[p][ele]:
                if mertic == "devices_io_raw":
                    for dev in PERFORMANCE_AUTODRIVE[p][ele][mertic]:
                        print(f"{tab}{tab}{tab}{dev}:")
                        for dev_metric in PERFORMANCE_AUTODRIVE[p][ele][mertic][dev]:
                            avg = round(
                                np.mean(PERFORMANCE_AUTODRIVE[p][ele][mertic][dev][dev_metric]), 2)
                            percentile_90 = round(
                                np.percentile(PERFORMANCE_AUTODRIVE[p][ele][mertic][dev][dev_metric], 90), 2)
                            print(f"{tab}{tab}{tab}{tab}{dev_metric}: \t[avg: {avg}]\t[90th: {percentile_90}]")
                else:
                    if len(PERFORMANCE_AUTODRIVE[p][ele][mertic]) == 0:
                        continue
                    avg = round(np.mean(PERFORMANCE_AUTODRIVE[p][ele][mertic]), 2)
                    percentile_90 = round(np.percentile(PERFORMANCE_AUTODRIVE[p][ele][mertic], 90), 2)
                    print(f"{tab}{tab}{tab}{mertic}: \t[avg: {avg}]\t[90th: {percentile_90}]")

def main():
    """ Main Method
    """
    global PERFORMANCE_FILE

    parser = argparse.ArgumentParser(
        description='performance results parse and upload tool')
    parser.add_argument("-f", "--file", required=True,
        nargs=1, type=str.lstrip, help="the performance file dumped by cyber_performance")

    args = parser.parse_args()
    
    PERFORMANCE_FILE = args.file[0]

    parse_result()


if __name__ == '__main__':
    main()
