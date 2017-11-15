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
Recorder and RecorderOptions.
"""

import os
import sys
import time
import Queue
import signal
import fnmatch
import logging
import datetime
import commands
import threading
import traceback
import subprocess

import roslib.packages
import meta_manager


class SubprocessStreamReader(object):
    """Subprocess Nonblocking Stream Reader."""
    def __init__(self, stream):
        '''Init: the stream to read from.'''
        self.s = stream
        self.q = Queue.Queue()
        def feed_queue(stream, queue):
            '''Put stream line into queue.'''
            while True:
                line = stream.readline()
                if line:
                    queue.put(line)
                time.sleep(1)
        self.t = threading.Thread(target = feed_queue,
                args = (self.s, self.q))
        self.t.daemon = True
        self.t.start()

    def readline(self, timeout = None):
        """Read stream from queue."""
        try:
            return self.q.get(timeout=timeout)
        except Queue.Empty:
            return None


class RecorderOptions(object):
    """Recorder Options"""
    def __init__(self):
        """Init"""
        self.record_all = False
        self.record_path = "./"
        self.record_quiet = True
        self.record_switch = True
        self.record_split = True
        self.record_buffer_size = 256
        self.record_chunk_size = 1024
        self.record_split_type = "duration"
        self.record_split_duration = "60"
        self.record_split_size = 300
        self.record_compress_type = "None"
        self.record_prefix = "rosbag" 
        self.record_topic_match_regex = "ddd"
        self.record_topic_exclude_regex = "dfdf"


class Recorder(threading.Thread):
    """Rosbag Recorder."""
    def __init__(self, recorder_manager, recorder_opts):
        """Init"""
        threading.Thread.__init__(self)
        self.exitcode = 0
        self.exception = None
        self.exc_traceback = None
        self.record_process = None
        self.ssr = None
        self.recorder_opts = recorder_opts
        self.recorder_manager = recorder_manager
        self.current_writing_disk = None
        self.writing_filename = None

    def construct_command(self):
        """Command Construction."""
        compress_arg = ""
        if_quiet_arg = ""
        split_arg = ""
        rosbag_output_prefix = self.recorder_opts.record_path \
                             + "/" \
                             + self.recorder_opts.record_prefix
        if self.recorder_opts.record_quiet:
            if_quiet_arg = "--quiet"
        if self.recorder_opts.record_compress_type == 'bz2':
            compress_arg = "--bz2"
        if self.recorder_opts.record_compress_type == 'lz4':
            compress_arg = "--lz4"
        if self.recorder_opts.record_split:
            if self.recorder_opts.record_split_type == 'duration':
                split_arg = "--split --duration " \
                          + str(self.recorder_opts.record_split_duration)
            elif self.recorder_opts.record_split_type == 'size':
                split_arg = "--split --size " \
                          + str(self.recorder_opts.record_split_size)
            else:
                logging.error("Invalid compression parameter, only support duration or size")
                return (-1, 'arguments error')

        recordpath = roslib.packages.find_node('rosbag', 'record')
        if not recordpath:
            logging.error("Cannot find rosbag/record executable")
            return(-1, 'Cannot find rosbag/record executable')
        cmd = recordpath[0] \
            + " " \
            + split_arg \
            + " " \
            + compress_arg \
            + " " \
            + "--buffsize " + str(self.recorder_opts.record_buffer_size) \
            + " " \
            + "--output-prefix " + rosbag_output_prefix \
            + " " \

        if self.recorder_opts.record_topic_match_regex == "":
            cmd = cmd \
                + "--all" \
                + " " \
                + "--exclude " + "\'" + self.recorder_opts.record_topic_exclude_regex + "\'" \
                + " " \
                + if_quiet_arg 
        else:
            cmd = cmd \
                + "--regex " + "\'" + self.recorder_opts.record_topic_match_regex + "\'" \
                + " " \
                + if_quiet_arg 
        return (0, cmd)

    def rosbag_record(self, cmd):
        """Record rosbag."""
        logging.info("Rosbag record subprocess starting, cmd=%s", cmd)
        self.record_process = subprocess.Popen(
                            cmd, 
                            shell=True, 
                            stdout=subprocess.PIPE, 
                            stderr=subprocess.STDOUT,
                            preexec_fn=os.setsid)
        logging.info("Rosbag record pid=%s", str(self.record_process.pid))
        #self.record_process.wait()
        self.ssr  = SubprocessStreamReader(self.record_process.stdout)

    def create_record(self):
        """Create new recorder."""
        return_code, cmd = self.construct_command()
        if return_code == 0:
            self.rosbag_record(cmd)

    def thread_run(self):
        """Do record."""
        self.create_record()
        while True: 
            time.sleep(1) 
            record_status_code = self.record_process.poll()
            if record_status_code is not None:
                os.killpg(self.record_process.pid, signal.SIGINT)
                self.exitcode = record_status_code
                break
            if not self.recorder_manager.record_enable:
                os.killpg(self.record_process.pid, signal.SIGINT)
                self.exitcode = -1024 # Stop record because of record has been disabled.
                break
            output = self.ssr.readline(0.1)
            if output is not None:
                logging.info("Record subprocess stream: %s", output)
                if output in ['Aborted\n', 'Terminated\n'] and self.exitcode != -1024:
                    os.killpg(self.record_process.pid, signal.SIGINT)
                    self.exitcode = -2
                    break
            if self.recorder_manager.stop_signal:
                logging.info("Catch stop signal and stop rosbag record.")
                print("Catch stop signal and stop rosbag record. record exit!")
                os.killpg(self.record_process.pid, signal.SIGINT)
                self.exitcode = 0
                break
        logging.info("Record exit: stdout=%s, stderr=%s, errorcode=%s",
            self.record_process.stdout, self.record_process.stderr, self.record_process.returncode)

    def run(self):
        """Thread run."""
        try:
            self.thread_run()
        except Exception as e:
            logging.error("Record process exit with exception.")
            self.exitcode = -1
            self.exception = e
            self.exc_traceback = ''.join(traceback.format_exception(*sys.exc_info()))
