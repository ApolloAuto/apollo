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

Run with '--help' to see more options.
"""

import argparse
import datetime
import os
import subprocess
import sys

import psutil

MAP_COLLECTION_DATA_TOPICS = [
    '/apollo/monitor/system_status',
    '/apollo/sensor/gnss/best_pose',
    '/apollo/sensor/gnss/gnss_status',
    '/apollo/sensor/gnss/imu',
    '/apollo/sensor/gnss/ins_stat',
    '/apollo/sensor/gnss/odometry',
    '/apollo/sensor/gnss/raw_data',
    '/tf',
    '/tf_static',
    '/apollo/sensor/camera/front_12mm/image/compressed',
    '/apollo/sensor/camera/front_6mm/image/compressed',
    '/apollo/sensor/lidar16/front/up/Scan',
    '/apollo/sensor/lidar16/front/up/PointCloud2',
    '/apollo/sensor/lidar16/front/up/compensator/PointCloud2',
    '/apollo/sensor/lidar128/PointCloud2',
    '/apollo/sensor/lidar128/compensator/PointCloud2',
    '/apollo/sensor/velodyne64/PointCloud2',
    '/apollo/sensor/velodyne64/compensator/PointCloud2',
    '/apollo/sensor/velodyne64/VelodyneScan',
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

    def __init__(self):
        """Manage disks."""
        disks = []
        for disk in psutil.disk_partitions():
            if not disk.mountpoint.startswith('/media/'):
                continue
            disks.append({
                'mountpoint': disk.mountpoint,
                'available_size': DiskManager.disk_avail_size(disk.mountpoint),
                'is_nvme': disk.mountpoint.startswith('/media/apollo/internal_nvme'),
            })
        # Prefer NVME disks and then larger disks.
        self.disks = sorted(
            disks, reverse=True,
            key=lambda disk: (disk['is_nvme'], disk['available_size']))

    @staticmethod
    def disk_avail_size(disk_path):
        """Get disk available size."""
        statvfs = os.statvfs(disk_path)
        return statvfs.f_frsize * statvfs.f_bavail


class Recorder(object):
    """Data recorder."""

    def __init__(self, args):
        self.args = args
        self.disk_manager = DiskManager()

    def start(self):
        """Start recording."""
        if Recorder.is_running():
            print('Another data recorder is running, skip.')
            return

        disks = self.disk_manager.disks
        # Use the best disk, or fallback '/apollo' if none available.
        disk_to_use = disks[0]['mountpoint'] if len(disks) > 0 else '/apollo'

        topics = list(MAP_COLLECTION_DATA_TOPICS)
        self.record_task(disk_to_use, topics)

    def stop(self):
        """Stop recording."""
        shell_cmd('pkill -f "cyber_recorder record"')

    def record_task(self, disk, topics):
        """Record tasks into the <disk>/data/bag/<task_id> directory."""
        task_id = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        task_dir = os.path.join(disk, 'data/bag', task_id)
        print('Recording bag to {}'.format(task_dir))

        log_file = '/apollo/data/log/apollo_record.out'
        topics_str = ' -c '.join(topics)

        os.makedirs(task_dir)
        cmd = '''
            cd "{}"
            source /apollo/scripts/apollo_base.sh
            source /apollo/framework/install/setup.bash
            nohup cyber_recorder record -c {} >{} 2>&1 &
        '''.format(task_dir, topics_str, log_file)
        shell_cmd(cmd)

    @staticmethod
    def is_running():
        """Test if the given process running."""
        _, stdout, _ = shell_cmd('pgrep -c -f "cyber_recorder record"', False)
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
