"""
cyber benchmark
"""
#!/usr/bin/env python3
# ****************************************************************************
# Copyright 2024 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ****************************************************************************

import os
import re
import sys
import json
import time
import numpy as np
import argparse
import atexit
import signal
import logging
import itertools
import subprocess
import threading
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler


"""
colorful logging
"""
BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE = list(range(8))
RESET_SEQ = "\033[0m"
COLOR_SEQ = "\033[1;%dm"
BOLD_SEQ = "\033[1m"

COLORS = {
    'INFO': GREEN,
    'WARNING': YELLOW,
    'DEBUG': BLUE,
    'ERROR': RED,
    'CRITICAL': YELLOW
}

logger = logging.Logger(__name__)
reader_nums = -1

class Benchmark(FileSystemEventHandler):
    """
    benchmark class definition
    """
    def __init__(self, params):
        self.params = params
        self.directory_to_watch = os.path.join(os.environ['APOLLO_ENV_WORKROOT'], "dumps")
        self.preformance_to_write = os.path.join(os.environ['APOLLO_ENV_WORKROOT'], "data")
        self.performances = {}
        self.performance_key = None
        self.debounce_period = 1
        self.last_sample_time = 0
        self.base_key = {
            "cyber_benchmark_cpu_usage_latency": "cpu_usage",
            "cyber_benchmark_mem_resident_usage_latency": "mem_usage",
        }
        self.latency_key = {}
        self.data_fd = open(
            "transport_performance_{}".format(int(time.time())), "w+")

    def on_any_event(self, event):
        """
        monitor the files associated with performance
        """
        if event.is_directory:
            return None

        elif event.event_type == 'created' or event.event_type == 'modified':
            if "cyber_benchmark_reader" not in event.src_path.split('/')[-1] or \
                    not event.src_path.split('/')[-1].endswith("latency.data"):
                return None
            if self.performance_key is None:
                return None

            current = time.time()
            if current - self.last_sample_time < 1:
                return None

            self.last_sample_time = current
            
            latancy_data_path = event.src_path
            try:
                with open(latancy_data_path, "r") as f:
                    for l in f.readlines():
                        k, v = l.strip().split(" : ")[0], l.strip().split(" : ")[1]
                        if k in self.latency_key:
                            if self.latency_key[k] not in self.performances[self.performance_key]:
                                self.performances[self.performance_key][self.latency_key[k]] = []  
                            self.performances[
                                self.performance_key][self.latency_key[k]].append(int(v))
            except:
                return None
    
    def parse_result(self):
        """
        get the final results of performance
        """
        global reader_nums
        for i in range(reader_nums):
            data_path = os.path.join(
                self.directory_to_watch, f"cyber_benchmark_reader_{i}.data")
            recv_total = -1
            send_total = -1
            with open(data_path, "r") as f:
                for l in f.readlines():
                    k, v = l.strip().strip().split(" : ")[0], l.strip().split(" : ")[1]
                    if k == f"cyber_benchmark_cyber_benchmark_reader_{i}_apollo_cyber_benchmark_recv_msgs_nums":
                        if i == 0:
                            recv_total = int(v)
                        else:
                            recv_total += int(v)
                    elif k == f"cyber_benchmark_cyber_benchmark_reader_{i}_apollo_cyber_benchmark_total_msgs_nums":
                        if i == 0:
                            send_total = int(v)
                        else:
                            send_total += int(v)
            if recv_total == -1 or send_total == -1:
                logger.error("parse cyber_benchmark_reader.data result failed")
                sys.exit(3)
        
        writer_data_path = os.path.join(
            self.directory_to_watch, "cyber_benchmark_writer.data")
        test_time = -1
        test_message_size = -1
        with open(writer_data_path, "r") as f:
            for l in f.readlines():
                k, v = l.strip().strip().split(" : ")[0], l.strip().split(" : ")[1]
                if k == "cyber_benchmark_cyber_benchmark_writer_test_time":
                    test_time = float(v)
                elif k == "cyber_benchmark_cyber_benchmark_writer_message_size":
                    test_message_size = int(v)
        if test_time == -1 or test_message_size == -1:
            logger.error("parse cyber_benchmark_writer.data result failed")
            sys.exit(3)

        test_message_size = test_message_size / 1024 / 1024 # in MB

        msg_lose_rate = round((send_total - recv_total) / send_total * 100, 2)
        bandwidth = round(recv_total * test_message_size / test_time * 8, 2) # mbps

        ret_str = f"{self.performance_key}:\n"
        for k in self.performances[self.performance_key]:
            self.performances[self.performance_key][k].remove(
                max(self.performances[self.performance_key][k])) 
            self.performances[self.performance_key][k].remove(
                min(self.performances[self.performance_key][k]))
            self.performances[self.performance_key][k] = list(
                filter(lambda x: x != 0, self.performances[self.performance_key][k]))
            if "cpu" in k:
                self.performances[self.performance_key][k] = list(
                    filter(lambda x: x < 120, self.performances[self.performance_key][k])) 
            pavg = round(np.mean(self.performances[self.performance_key][k]), 2)
            p99 = round(np.percentile(self.performances[self.performance_key][k], 99), 2)
            p95 = round(np.percentile(self.performances[self.performance_key][k], 95), 2)
            p50 = round(np.percentile(self.performances[self.performance_key][k], 50), 2)
            pmin = round(np.min(self.performances[self.performance_key][k]), 2)
            pmax = round(np.max(self.performances[self.performance_key][k]), 2)
            ret_str += f"{k}: \n"
            ret_str += f"  avg: {pavg}, pmin: {pmin}, pmax: {pmax}, p50: {p50}, p95: {p95}, p99: {p99}\n"

        ret_str += f"bandwidth: {bandwidth} mbps\n"
        ret_str += f"msg_lose_rate: {msg_lose_rate}%"
        print(ret_str)
        self.performances[self.performance_key]["test_message_size"] = test_message_size
        self.performances[self.performance_key]["test_time"] = test_time

    def save_data(self):
        """
        record all performance data
        """
        self.data_fd.write(
            json.dumps(self.performances[self.performance_key]))
        self.data_fd.flush()

    def run(self):
        """
        run the performance test
        """
        global reader_nums
        transport_data_type = []
        if self.params.transport_type == 0 or \
                self.params.transport_type == 1:
            transport_data_type = [self.params.transport_type]
        else:
            transport_data_type = [0, 1]

        cartesian_product = list(itertools.product(self.params.data_size,
            self.params.reader_nums, self.params.frequency, transport_data_type))

        for item in cartesian_product:
            ds = item[0]
            rn = item[1]
            f = item[2]
            dt = item[3]

            reader_nums = rn

            for k in self.base_key:
                self.latency_key[k] = self.base_key[k]
            for i in range(rn):
                self.latency_key[
                    f"cyber_benchmark_cyber_benchmark_reader_{i}_apollo_cyber_benchmark_cyber_latency"
                ] = "cyber_latency"
                self.latency_key[
                    f"cyber_benchmark_cyber_benchmark_reader_{i}_apollo_cyber_benchmark_tran_latency"
                ] = "cyber_tran_latency" 

            if self.params.best_effords:
                qos = 1
            else:
                qos = 0
            self.performance_key = \
                f"data-size:{ds}/reader-nums:{rn}/frequency:{f}/data_type:{dt}/qos:{qos}"
            self.performances[self.performance_key] = {}
            print(f"\nrunning test with {self.performance_key}")
            self.readers = []
            for i in range(rn):
                try:
                    reader_args_list = [
                        "cyber_benchmark_reader", 
                        "-n", f"{i}"]
                    reader = subprocess.Popen(reader_args_list, stdout=subprocess.DEVNULL,
                                                        stderr=subprocess.DEVNULL)
                except Exception as err:
                    logger.error('Subprocess Popen exception: ' + str(err))
                    sys.exit(2)
                else:
                    if reader.pid == 0 or reader.returncode is not None:
                        logger.error('Start process cyber_benchmark_reader failed.')
                        sys.exit(2)
                    self.readers.append(reader)
            time.sleep(3)
            try:
                writer_args_list = [
                    "cyber_benchmark_writer", "-s", f"{ds}",
                    "-t", f"{f}", "-T", f"{self.params.time}",
                    "-d", f"{dt}", "-q", f"{qos}"]
                ret = subprocess.run(writer_args_list, stdout=subprocess.DEVNULL,
                                        stderr=subprocess.DEVNULL)
            except Exception as err:
                logger.error('Subprocess run writer exception: ' + str(err))
                sys.exit(2)
            else:
                if ret.returncode != 0:
                    logger.error('Run process cyber_benchmark_writer failed.')
                    sys.exit(2)
            time.sleep(3)
            for reader in self.readers:
                reader.kill()
            self.parse_result()
            self.save_data()


def param_parse(params):
    """
    parse user parameters
    """
    pattern = re.compile(r'^\d+(\.\d+)?[kKmMbB]$')
    for i in range(len(params.data_size)):
        param = params.data_size[i]
        if not pattern.match(param):
            logger.error(f"data size {param} is invalid")
        params.data_size[i] = \
            param[: len(params.data_size[i]) - 1] + param[-1].upper()
    for i in range(len(params.reader_nums)):
        try:
            param = int(params.reader_nums[i])
        except:
            logger.error(
                f"reader nums {params.reader_nums[i]} is invalid")
        params.reader_nums[i] = param
    for i in range(len(params.frequency)):
        try:
            param = int(params.frequency[i])
        except:
            logger.error(
                f"frequency {params.frequency[i]} is invalid")
        params.frequency[i] = param 
    return params
        

def main():
    """
    entry
    """
    parser = argparse.ArgumentParser(description='cyber benchmark')
    parser.add_argument(
        '-s', '--data_size', nargs='*', metavar='*',
        default=['16B', '1K', '64K', '256K', '1M', '5M', '10M'],
        type=str.lstrip, help="transport data size, default is 16B - 10M"
    )
    parser.add_argument(
        '-T', '--transport_type', type=int, default=0,
        help='transport data type, 0 means bytes, 1 means repeted field, while others mean both'
    )
    parser.add_argument(
        '-b', '--best_effords', action='store_true', default=False,
        help='test the performance using qos policy best effords'
    )
    parser.add_argument(
        '-r', '--reader_nums', nargs='*', metavar='*', default=[1, 2, 5, 10],
        help='desribe how many readers receive the message'
    )
    parser.add_argument(
        '-f', '--frequency', nargs='*', metavar='*', default=[10, 20, 50, 100, -1],
        help='desribe the frequency of message sended, e.g. 10 means 10 hz'
    )
    parser.add_argument(
        '-t', '--time', type=int, default=60,
        help='desribe the time of performance test, default is 60s'
    )
    params = param_parse(parser.parse_args(sys.argv[1:]))
    benchmark = Benchmark(params)
    observer = Observer()
    
    observer.schedule(benchmark, benchmark.directory_to_watch)
    observer.start()
    benchmark.run()

if __name__ == "__main__":
    main()