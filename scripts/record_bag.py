#!/usr/bin/env python3

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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
Start apollo data recorder.

It lists all available disks mounted under /media, and prioritize them in order:
   - Disk#1. Largest NVME disk
   - Disk#2. Smaller NVME disk
   - ...
   - Disk#x. Largest Non-NVME disk
   - Disk#y. Smaller Non-NVME disk
   - ...

1. If we have two or more disks, LARGE_TOPICS will be recorded to Disk#1, and
   SMALL_TOPICS goes to Disk#2.
2. If we have only one NVME disk, it will be used for both LARGE and SMALL
   topics.
3. If we have only one Non-NVME disk, it will only record SMALL_TOPICS. But if
   you specify '--all', both LARGE and SMALL topics will be recorded.
4. If no external disks are available, we will take '/apollo' as a
   'Non-NVME disk' and follow the rule above.

Run with '--help' to see more options.
"""

import argparse
import datetime
import os
import subprocess
import sys

import psutil


LARGE_TOPICS = [
    '/apollo/sensor/camera/obstacle/front_6mm',
    '/apollo/sensor/camera/traffic/image_long',
    '/apollo/sensor/camera/traffic/image_short',
    '/apollo/sensor/velodyne64/compensator/PointCloud2',
]
SMALL_TOPICS = [
    '/apollo/canbus/chassis',
    '/apollo/canbus/chassis_detail',
    '/apollo/control',
    '/apollo/control/pad',
    '/apollo/drive_event',
    '/apollo/localization/pose',
    '/apollo/localization/msf_gnss',
    '/apollo/localization/msf_lidar',
    '/apollo/localization/msf_status',
    '/apollo/monitor',
    '/apollo/monitor/static_info',
    '/apollo/monitor/system_status',
    '/apollo/navigation',
    '/apollo/perception/obstacles',
    '/apollo/perception/traffic_light',
    '/apollo/planning',
    '/apollo/prediction',
    '/apollo/relative_map',
    '/apollo/routing_request',
    '/apollo/routing_response',
    '/apollo/sensor/camera/obstacle/front_6mm',
    '/apollo/sensor/conti_radar',
    '/apollo/sensor/delphi_esr',
    '/apollo/sensor/gnss/best_pose',
    '/apollo/sensor/gnss/corrected_imu',
    '/apollo/sensor/gnss/gnss_status',
    '/apollo/sensor/gnss/imu',
    '/apollo/sensor/gnss/ins_stat',
    '/apollo/sensor/gnss/odometry',
    '/apollo/sensor/gnss/raw_data',
    '/apollo/sensor/gnss/rtk_eph',
    '/apollo/sensor/gnss/rtk_obs',
    '/apollo/sensor/mobileye',
    '/tf',
    '/tf_static',
]

def shell_cmd(cmd, alert_on_failure=True):
    """Execute shell command and return (ret-code, stdout, stderr)."""
    print('SHELL > {}'.format(cmd))
    proc = subprocess.Popen(cmd, shell=True, close_fds=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    ret = proc.wait()
    stdout = proc.stdout.read().decode('utf-8') if proc.stdout else None
    stderr = proc.stderr.read().decode('utf-8') if proc.stderr else None
    if alert_on_failure and stderr and ret != 0:
        sys.stderr.write('{}\n'.format(stderr))
    return (ret, stdout, stderr)


class ArgManager(object):
    """Arguments manager."""

    def __init__(self):
        self.parser = argparse.ArgumentParser(
            description="Manage apollo data recording.")
        self.parser.add_argument('--start', default=False, action="store_true",
                                 help='Start recorder. It is the default '
                                 'action if no other actions are triggered. In '
                                 'that case, the False value is ignored.')
        self.parser.add_argument('--stop', default=False, action="store_true",
                                 help='Stop recorder.')
        self.parser.add_argument('--all', default=False, action="store_true",
                                 help='Record all topics even without high '
                                 'performance disks.')
        self.parser.add_argument('--split_duration', default="1m",
                                 help='Duration to split bags, will be applied '
                                 'as parameter to "rosbag record --duration".')
        self._args = None

    def args(self):
        """Get parsed args."""
        if self._args is None:
           self._args = self.parser.parse_args()
        return self._args


class DiskManager(object):
    """Disk manager."""

    def __init__(self, args):
        """Manage disks."""
        self.args = args

        disks = []
        for disk in psutil.disk_partitions():
            if not disk.mountpoint.startswith('/media/'):
                continue
            disks.append({
                'mountpoint': disk.mountpoint,
                'available_size': DiskManager.disk_avail_size(disk.mountpoint),
                'is_nvme': disk.device.startswith('/dev/nvme'),
            })
        # Prefer NVME disks and then larger disks.
        self.disks = sorted(
            disks, reverse=True,
            key=lambda disk: (disk['is_nvme'], disk['available_size']))

    def disk_for_large_topics(self):
        """Return a disk for recording large topics."""
        # We have a NVME disk, return it for large topics.
        if len(self.disks) > 0 and self.disks[0]['is_nvme']:
            return self.disks[0]['mountpoint']
        # If we are requested to record all topics, return the largest portable
        # disk, or fall back to the Apollo home directory.
        if self.args.all:
            if len(self.disks) > 0:
                return self.disks[0]['mountpoint']
            else:
                return '/apollo'
        return None

    def disk_for_small_topics(self):
        """Return a disk for recording small topics."""
        # If we have multiple disks, use the second-best one for small topics.
        if len(self.disks) > 1:
            return self.disks[1]['mountpoint']
        # If we have only one portable disk, use it for small topics. Or else
        # fall back to the Apollo home directory.
        return self.disks[0]['mountpoint'] if len(self.disks) > 0 else '/apollo'

    @staticmethod
    def disk_avail_size(disk_path):
        """Get disk available size."""
        statvfs = os.statvfs(disk_path)
        return statvfs.f_frsize * statvfs.f_bavail


class Recorder(object):
    """Data recorder."""

    def __init__(self, args):
        self.args = args
        self.disk_manager = DiskManager(args)

    def start(self):
        """Start recording."""
        if Recorder.is_running():
            print('Another data recorder is running, skip.')
            return

        task_id = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        large_topics_disk = self.disk_manager.disk_for_large_topics()
        small_topics_disk = self.disk_manager.disk_for_small_topics()
        if large_topics_disk == small_topics_disk:
            self.record_task(large_topics_disk, LARGE_TOPICS + SMALL_TOPICS,
                             task_id, 'all')
        else:
            if large_topics_disk:
                self.record_task(large_topics_disk, LARGE_TOPICS,
                                 task_id, 'large')
            self.record_task(small_topics_disk, SMALL_TOPICS, task_id, 'small')

    def stop(self):
        """Stop recording."""
        shell_cmd('kill -INT $(pgrep -f "rosbag/record" | grep -v pgrep)')

    def record_task(self, disk, topics, task_id, task_type):
        """Record tasks into the <disk>/data/bag/<task_id> directory."""
        task_dir = os.path.join(disk, 'data/bag', task_id)
        # As we are using the same task_id, LARGE_TOPICS will goes to subfolder
        # to avoid conflict when merging.
        if task_type == 'large':
            task_dir = os.path.join(task_dir, task_type)
        print('Recording {} topics to {}'.format(task_type, task_dir))

        log_file = '/apollo/data/log/apollo_record_{}.out'.format(task_type)
        topics_str = ' '.join(topics)

        os.makedirs(task_dir)
        cmd = '''
            cd "{}"
            nohup rosbag record --split --duration={} -b 2048 {} >{} 2>&1 &
        '''.format(task_dir, self.args.split_duration, topics_str, log_file)
        shell_cmd(cmd)

    @staticmethod
    def is_running():
        """Test if the given process running."""
        _, stdout, _ = shell_cmd('pgrep -c -f "rosbag/record"', False)
        # If stdout is the pgrep command itself, no such process is running.
        return stdout.strip() != '1' if stdout else False


def main():
    """Main entry."""
    arg_manager = ArgManager()
    args = arg_manager.args()
    recorder = Recorder(args)
    if args.stop:
        recorder.stop()
    else:
        recorder.start()

if __name__ == '__main__':
    main()
