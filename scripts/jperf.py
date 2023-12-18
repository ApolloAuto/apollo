#!/usr/bin/env python3

###############################################################################
# Copyright 2023 The Apollo Authors. All Rights Reserved.
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
performance calculation script of orin
"""
import os
import time
import numpy
import atexit
import signal
import argparse
import threading
import subprocess
from pathlib import Path

DEFAULT_PATH = "{}/performance".format(os.path.dirname(os.path.abspath(__file__)))
FILENAME_PREFIX = "orin"
TIME_SUFFIX = int(time.time())

def ProcessSingleLine(line):
    """
    process tegrastats record performance
    """
    elements = line.split(" ")
    mem_used = int(elements[3].split("/")[0])
    mem_total = int(elements[3].split("/")[1][0: len(elements[3].split("/")[1])-2])
    mem_usage = mem_used / mem_total
    cpu_raw_string_list = elements[11].split(",")
    cpu_raw_string_list[0] = cpu_raw_string_list[0][1:] 
    cpu_raw_string_list[-1] = cpu_raw_string_list[-1][: len(cpu_raw_string_list[-1])-1]
    cpu_usage_list = []
    for cpu_raw_string in cpu_raw_string_list:
        cpu_usage_list.append(int(cpu_raw_string.split("%@")[0]))
    
    cpu_usage_avg = numpy.mean(cpu_usage_list)
    gpu_usage = int(elements[15][0: len(elements[15])-1])
    return cpu_usage_avg, gpu_usage, mem_usage

def ExitSignalHandler(sig, action):
    """
    terminate subprocess
    """
    global P
    if P is not None:
        P.terminate()

def AtExitFunc():
    """
    calculate the performance when quit
    """
    cpu_list = []
    gpu_list = []
    mem_list = []
    for filename in os.listdir(DEFAULT_PATH):
        if filename.startswith("{}.{}".format(FILENAME_PREFIX, TIME_SUFFIX)):
            with open(
                "{}/{}.{}".format(DEFAULT_PATH, FILENAME_PREFIX, TIME_SUFFIX), 
                "r") as f:
                for line in f.readlines():
                    cpu, gpu, mem = ProcessSingleLine(line)
                    cpu_list.append(cpu)
                    gpu_list.append(gpu)
                    mem_list.append(mem)
    if len(cpu_list) == 0 or len(gpu_list) == 0 or len(mem_list) == 0:
        exit(0)
    print("result:")
    print("gpu: %.2f\ncpu:%.2f\nmem:%.2f" % (
        numpy.mean(gpu_list), numpy.mean(cpu_list), numpy.mean(mem_list) * 100))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='performance calculation of jetson')
    parser.add_argument(
        '-n', '--name', default="orin", type=str.lstrip, 
        help='name of performance file' 
    )
    atexit.register(AtExitFunc)
    signal.signal(signal.SIGINT, ExitSignalHandler)
    signal.signal(signal.SIGTERM, ExitSignalHandler)

    if Path(DEFAULT_PATH).exists():
        if Path(DEFAULT_PATH).is_symlink() or Path(DEFAULT_PATH).is_file():
            print("{} is not a dictionary".format(DEFAULT_PATH))
            exit(-1)
    else:
        os.makedirs(DEFAULT_PATH, exist_ok=True)
    
    parser.parse_args()
    
    FILENAME_PREFIX = parser.parse_args().name
    P = subprocess.Popen(
        [
            "tegrastats", 
            "--logfile", 
            "{}/{}.{}".format(DEFAULT_PATH, FILENAME_PREFIX, TIME_SUFFIX)
        ],
        shell=False
    )
    P.wait()
