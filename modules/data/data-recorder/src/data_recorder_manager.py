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
data_recorder_manager is responsiable for autonomous vehicle data recording.
"""

import os
import re
import sys
import time
import json
import signal
import Queue
import ctypes
import shutil
import inspect
import logging
import fnmatch
import commands
import datetime
import optparse
import traceback
import threading
import subprocess
import ConfigParser

import rospy
from std_msgs.msg import String

import recorder
import data_sync
import disk_handle
import meta_manager
import config_parser
from proto import recorder_info_pb2

G_VERSION = "1.0.0.1"

class DataRecorderManager(object):
    """Data Recorder Manager"""
    def __init__(self, cf_reader):
        """Recorder Init."""
        self.pub = None
        self.can_kill = False
        self.cmd_topic = None
        self.worker_list = []
        self.recorder_list = []
        self.sync_enable = True
        self.record_enable = True
        self.status_topic = None
        self.stop_signal = False
        self.latest_cmdtime = None
        self.recorder_status = None
        self.output_directory = None
        self.backup_directory = None
        self.recorder_opts_list = []
        self.rlock = threading.RLock()
        self.if_taskid_isready = False
        self.conf_reader = cf_reader
        self.init()

    def init(self):
        self.recorder_status = recorder_info_pb2.DATA_RECORDER_INIT
        self.cmd_topic = self.conf_reader.data_args.get("recorder_cmd_topic")
        self.status_topic = self.conf_reader.data_args.get("recorder_status_topic")
        self.pub = rospy.Publisher(self.status_topic, String, queue_size=100)
        signal.signal(signal.SIGINT, self.shutdown_hook)
        signal.signal(signal.SIGTERM, self.shutdown_hook)
        signal.signal(signal.SIGHUP, self.shutdown_hook)
        signal.signal(signal.SIGQUIT, self.shutdown_hook)

    def create_backup_id(self):
        """Create backup id according to system uptime."""
        backup_path = self.conf_reader.data_args.get('backup_path')
        if not os.path.exists(backup_path):
            try:
                os.mkdir(backup_path)
            except Exception as e:
                logging.error("Create %s failed, %s", backup_path, str(e))
                return -1
        backup_path = backup_path + "/" + self.get_system_uptime()
        if not os.path.exists(backup_path):
            try:
                os.mkdir(backup_path)
            except Exception as e:
                logging.error("Create backup id failed, %s", str(e))
                return -1
        self.backup_directory = backup_path
        logging.info("Create backup id %s successfully", backup_path)

    def create_task_id(self):
        """Recorder init, create_task_id."""
        if disk_handle.check_disk(self.conf_reader.data_args.get('output_path')) == -2:
            return -1
        self.create_backup_id()
        self.output_directory = self.conf_reader.data_args.get('output_path') \
                + '/' \
                + self.conf_reader.vehicle['vehicle_id'] \
                + '_' \
                + datetime.datetime.now().strftime('%Y%m%d%H%M%S')
        if not os.path.exists(self.output_directory):
            os.mkdir(self.output_directory)
        for data_dir in self.conf_reader.data_type:
            try:
                os.mkdir(self.output_directory + "/" + data_dir)
            except Exception as e:
                logging.error("Make sub directory in task failed, %s", str(e))
                return -1
        recorder_meta = {
            'meta_info': [
                {
                    'basic': {
                        'data_recorder_version': G_VERSION,
                        'organization_name': self.conf_reader.organization.get('name'),
                        'organization_website': self.conf_reader.organization.get('website'),
                        'organization_description': self.conf_reader.organization.get('description'),
                        'vehicle_id': self.conf_reader.vehicle.get('vehicle_id'),
                        'vehicle_type': self.conf_reader.vehicle.get('vehicle_type'),
                        'vehicle_tag': self.conf_reader.vehicle.get('vehicle_tag'),
                        'vehicle_description': self.conf_reader.vehicle.get('description'),
                        'task_purpose': self.conf_reader.task_purpose,
                        'system_uptime': self.get_system_uptime()
                    }
                }
            ]
        }
        meta_ext = self.conf_reader.data_args.get('meta_extension')
        meta_extension = (meta_ext
            if meta_ext == 'ini' or meta_ext == 'json' else "ini")
        ret = meta_manager.create_meta(self.output_directory + '/meta/recorder.' + meta_extension,
                meta_extension,
                recorder_meta)
        if ret != 0:
            return -1
        self.update_link()
        logging.info("Create task_id %s successfully", self.output_directory)
        for k, v in self.conf_reader.task_data_args.iteritems():
            if ['if_record'] and ['record_method'] == "rsync":
                if ['action_args']['with_remove']:
                    continue
                self.sync_static_data(k)
        self.if_taskid_isready = True
        return 0

    def listener(self):
        """Listening specific topic."""
        rospy.Subscriber(self.cmd_topic, String, self.listener_callback)

    def listener_callback(self, data):
        """listener callback."""
        logging.info("receive message from %s, data=%s", self.cmd_topic, data.data)
        if self.latest_cmdtime is not None: 
            if (datetime.datetime.now() - self.latest_cmdtime) < datetime.timedelta(seconds=60):
                print_message = (
                    "The interval between two consecutive operation"
                     " must not be less than 60 seconds! Thanks!"
                )
                print("\33[1;35;2m%s\033[0m" % (print_message))
                return
        self.latest_cmdtime = datetime.datetime.now() 
        if data.data == "rosbag_record_on" and self.rlock.acquire():
            self.record_enable = True
            self.recorder_status |= recorder_info_pb2.DATA_RECORD_ENABLE
            self.rlock.release()
            print("\33[1;31;2mRosbag record has been enabled!\033[0m")
            return
        elif data.data == "rosbag_record_off" and self.rlock.acquire():
            self.record_enable = False
            self.recorder_status &= (~recorder_info_pb2.DATA_RECORD_ENABLE)
            self.rlock.release()
            print("\33[1;31;2mRosbag record has been disabled!\033[0m")
            return
        elif data.data == "data_sync_on" and self.rlock.acquire():
            self.sync_enable = True
            self.recorder_status |= recorder_info_pb2.DATA_SYNC_ENABLE
            self.rlock.release()
            print("\33[1;31;2mData sync has been enabled!\033[0m")
            return
        elif data.data == "data_sync_off" and self.rlock.acquire():
            self.sync_enable = False
            self.recorder_status &= (~recorder_info_pb2.DATA_SYNC_ENABLE)
            self.rlock.release()
            print("\33[1;31;2mData sync has been disabled!\033[0m")
            return
        else:
            print_message = (
                "Invalid command! please try again after 60 seconds."
            )
            print("\33[1;35;2m%s\033[0m" % print_message)
        
    def sync_static_data(self, data):
        """Sync data."""
        src = self.conf_reader.task_data_args[data]['data_property']['src']
        dst = self.conf_reader.task_data_args[data]['data_property']['dst']
        limit = self.conf_reader.task_data_args[data]['action_args']['sync_bwlimit']
        src = src if src.endswith('/') else src + "/"
        dst = self.output_directory + "/" + dst + "/"
        cmd = "mkdir -p " + dst \
            + " && /usr/bin/rsync -rcC --bwlimit=" \
            + str(limit) \
            + " " + src + " " + dst
        process = subprocess.Popen(
            cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE)
        out, err = process.communicate()
        errcode = process.returncode
        if errcode == 0:
            logging.info("Sync %s to %s successfully after creating task_id", src, dst)
        else:
            logging.error("Sync data failed after creating task_id, \
                cmd=%s, stdout=%s, stderr=%s, errcode=%s", cmd, out, err, errcode)

    def publish_recorder_info(self, event):
        """Publish data recorder status."""
        if not rospy.is_shutdown():
            info = recorder_info_pb2.RecorderInfo()
            task = recorder_info_pb2.Task()
            task_id = os.path.split(os.path.abspath(self.output_directory))[1]
            task.id = task_id
            datetime_task_start = datetime.datetime.strptime(task_id.split('_')[1], '%Y%m%d%H%M%S')
            task.duration = (datetime.datetime.now() - datetime_task_start).seconds
            resources = recorder_info_pb2.Resources()
            disk_partitions = disk_handle.get_disk_partitions_info()
            for dp in disk_partitions:
                db_usage = disk_handle.get_disk_usage_info(dp.mountpoint)
                disk = resources.disks.add()
                disk.fs = dp.fs
                disk.sn = "UNKNOWN"
                disk.slot = "UNKNOWN"
                disk.size = float(db_usage.total)
                disk.used = float(db_usage.used)
                disk.avail = float(db_usage.free)
                disk.use_percent = float(db_usage.percent)
                disk.mount = dp.mountpoint
                if disk.mount == disk_handle.get_mount_point(self.conf_reader.data_args.get('output_path')):
                    info.writing_disk.CopyFrom(disk)
                    if disk_handle.check_disk(self.conf_reader.data_args.get('output_path')) == 0:
                        self.recorder_status &= (~recorder_info_pb2.DISK_SPACE_WARNNING)
                        self.recorder_status &= (~recorder_info_pb2.DISK_SPACE_ALERT)
                    if disk_handle.check_disk(self.conf_reader.data_args.get('output_path')) == -1:
                        self.recorder_status |= recorder_info_pb2.DISK_SPACE_WARNNING
                    if disk_handle.check_disk(self.conf_reader.data_args.get('output_path')) == -2:
                        self.recorder_status |= recorder_info_pb2.DISK_SPACE_WARNNING
                        self.recorder_status |= recorder_info_pb2.DISK_SPACE_ALERT
                        if self.rlock.acquire():
                            self.record_enable = False
                            self.sync_enable = False
                            self.rlock.release()
                     
                    if not self.sync_enable and self.rlock.acquire():
                        self.recorder_status &= (~recorder_info_pb2.DATA_SYNC_ENABLE)
                        self.rlock.release()
            data = recorder_info_pb2.Data()
            info.status = self.recorder_status
            info.task.CopyFrom(task)
            info.data.CopyFrom(data)
            info.resources.CopyFrom(resources)
            serialized_str = info.SerializeToString()
            self.pub.publish(serialized_str)

    def update_link(self):
        """Update softlink target file."""
        output_link_path  = self.conf_reader.data_args.get('output_link_path')
        try:
            os.remove(output_link_path)
            logging.info("Remove link file succeed")
        except Exception as e:
            logging.error("Remove link file failed" + str(e))
        try:
            os.symlink(os.path.abspath(self.output_directory), output_link_path)
        except Exception as e:
            logging.error("Update link file failed " + str(e))

    def get_system_uptime(self):
        """Get system startup time."""
        cmd = "uptime -s|sed 's/[-: ]//g'"
        ret, rst =  commands.getstatusoutput(cmd)
        if ret == 0:
            return rst
        else:
            return None

    def start_recorder(self):
        """Create a recorder and record data to current_output_path."""
        self.create_task_id()
        if not self.if_taskid_isready:
            return -1
        rosbag_args  = self.conf_reader.task_data_args['rosbag']
        rosbag_dst = rosbag_args['data_property']['dst']
        rosbag_buffer_size = rosbag_args['action_args']['rosbag_buffer_size']
        rosbag_chunk_size = rosbag_args['action_args']['rosbag_chunk_size']
        rosbag_split = rosbag_args['action_args']['rosbag_split']
        rosbag_split_duration = rosbag_args['action_args']['rosbag_split_duration']
        rosbag_compress_type = rosbag_args['action_args']['rosbag_compress_type']
        rosbag_groups = rosbag_args['action_args']['rosbag_topic_group']
        rosbag_path = self.output_directory + "/" + rosbag_dst
        record_instance = ()
        for group in rosbag_groups:
            group_id = group['group_id']
            group_name = group['group_name']
            group_topic_match_regex = group['group_topic_match_re']
            group_topic_exclude_regex = group['group_topic_exclude_re']
            prefix = "rosbag_" + self.conf_reader.vehicle['vehicle_id'] + "_" + group_name
            opts = recorder.RecorderOptions()
            opts.record_path = rosbag_path 
            opts.record_prefix = prefix
            opts.record_quiet = True
            opts.record_switch = True
            opts.record_buffer_size = rosbag_buffer_size
            opts.record_chunk_size = rosbag_chunk_size
            opts.record_compress_type = rosbag_compress_type
            opts.record_split = True
            opts.record_split_duration = rosbag_split_duration
            opts.record_prefix = prefix   
            opts.record_topic_match_regex = group_topic_match_regex
            opts.record_topic_exclude_regex = group_topic_exclude_regex
            record = recorder.Recorder(self, opts)
            record_instance = (group_name, record, opts)
            self.recorder_list.append(record_instance)
            self.recorder_opts_list.append((group_name, opts))
            self.worker_list.append(record)
        sync_thread = data_sync.DataSync(self)
        self.worker_list.append(sync_thread)
        for worker in self.worker_list:
            worker.setDaemon(False)
            worker.start()
        self.recorder_status |= 7 # running & record & sync
        self.listener()
        
        timer_publish = rospy.Timer(rospy.Duration(2), self.publish_recorder_info)
        while not rospy.is_shutdown():
            rospy.sleep(1)
            is_alive = False
            if self.record_enable and len(self.recorder_list) == 0:
                for group_name, opts in self.recorder_opts_list:
                    record = recorder.Recorder(self, opts)
                    record_instance = (group_name, record, opts)
                    record.start()
                    self.recorder_list.append(record_instance)
            for index, ins in enumerate(self.recorder_list):
                record_name, record_pid, record_opts = self.recorder_list[index]
                if record_pid.exitcode == -1024:
                    del self.recorder_list[index]
                    break
                if record_pid.exitcode is not None and not record_pid.is_alive():
                    logging.warn("Record group %s is not alive, try to restart.", record_name)
                    try:
                        record_pid.start()
                    except Exception as e:
                        logging.warn("Record group %s is not alive, restart, %s", record_name, str(e))
                if record_pid.exitcode is not None and record_pid.exitcode != 0:
                    logging.info("Record exit, name=%s, exitcode=%s", 
                        record_name, 
                        str(record_pid.exitcode))
                    logging.warn("Record group %s is terminated with exception, try to restart.", record_name)
                    record = recorder.Recorder(self, record_opts)
                    record.start()
                    del self.recorder_list[index]
                    self.recorder_list.append((record_name, record, record_opts))
            for worker in self.worker_list:
                is_alive = is_alive or worker.isAlive()
            if not is_alive:
                break
            if self.can_kill:
                self.stop_signal = True
                break
        self.recorder_status = recorder_info_pb2.DATA_RECORDER_EXIT
        timer_publish.shutdown()
        for worker in self.worker_list:
            worker.join()
        return 0

    def shutdown_hook(self, signum, frame):
        """Handle signal."""
        logging.info("Catch signal, signum=%s", str(signum))
        self.can_kill = True


def print_version(ver):
    """Print the version of program."""
    print("Program:data_recorder\nversion: %s") % (ver)
    return 0


def launch_data_recorder(cp):
    """Launch data recorder."""
    try:
        recorder_manager = DataRecorderManager(cp)
        recorder_manager.start_recorder()
    except KeyboardInterrupt:
        print("Receive ctrl+c and exit.")
        sys.exit(-1)
    logging.info("Exit data-recorder.")
    print("Exit data-recorder successfully.")

    return 0


def main():
    """main function"""
    usage = "python data_recorder.py -c ../conf/recorder.conf"
    parser = optparse.OptionParser(usage)
    parser.add_option("-c", "--conf",
            dest = "conf_file",
            help = "Specific data recorder config file")
    parser.add_option("-v", "--version",
            action = "store_true",
            dest = "my_version",
            default = False,
            help = "Show the version of this program")
    (options, args) = parser.parse_args()

    if len(sys.argv) == 1:
        parser.error("Incorrect numbers of agruments")
    if len(args):
        parser.error("Incorrect numbers of agruments, please type python data_recorder.py -h")

    if options.my_version:
        return print_version(G_VERSION)

    if options.conf_file is not None:
        if not os.path.exists(options.conf_file):
            parser.error("The config file you given does not exists, please check!")
        else:
            cp =config_parser.ConfigParser()
            global_conf = cp.load_config("../conf/recorder.global.yaml")
            task_conf = cp.load_config(options.conf_file)
            if global_conf is None:
                print("Load recorder.global.yaml error!")
                sys.exit(-1)
            if task_conf is None:
                print("Load recorder.debug.yaml error!")
                sys.exit(-1)
            if not cp.get_global_config(global_conf) == 0:
                print("Get global parameters from ../conf/recorder.global.yaml encounters error!")
                sys.exit(-1)
            if not cp.get_task_from_yaml(task_conf) == 0:
                print("Get task parameters from %s encounters error!" % (options.conf_file))
            rospy.init_node('data_recorder', anonymous=False)
            launch_data_recorder(cp)

    return 0


if __name__ == '__main__':
    """
    Run You.
    """
    main()
