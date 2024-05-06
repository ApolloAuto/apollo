#!/usr/bin/env python3

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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
Start apollo smart recorder.

It lists all available disks mounted under /media, and prioritize them in order:
   - Disk#1. Largest NVME disk
   - Disk#2. Smaller NVME disk
   - ...
   - Disk#x. Largest Non-NVME disk
   - Disk#y. Smaller Non-NVME disk
   - /apollo. If no external disks are available

Run with '--help' to see more options.
"""

import argparse
import datetime
import os
import subprocess
import sys

import psutil

from pathlib import Path


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
        """Init."""
        self.parser = argparse.ArgumentParser(
            description="Manage apollo data recording.")
        self.parser.add_argument('--start', default=False, action="store_true",
                                 help='Start recorder. It is the default '
                                 'action if no other actions are triggered. In '
                                 'that case, the False value is ignored.')
        self.parser.add_argument('--stop', default=False, action="store_true",
                                 help='Stop recorder.')
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
        """Init."""
        self.args = args
        self.disk_manager = DiskManager()

    def start(self):
        """Start recording."""
        if Recorder.is_running():
            Recorder.stop(self)
            print('Another smart recorder is running, stop now.')
        disks = self.disk_manager.disks
        disk_to_use = disks[0]['mountpoint'] if len(disks) > 0 else '/apollo'
        self.record_task(disk_to_use)

    def stop(self):
        """Stop recording."""
        shell_cmd('pkill --signal=SIGINT -f "smart_recorder"')

    def record_task(self, disk):
        """
        Save the full data into <disk>/data/bag/ReusedRecordsPool,
        which will be cleared every time the smart recorder get started.
        Meanwhile, restore the messages we are interested in to <disk>/data/bag/<task_id> directory.
        """
        flags_package_management = False
        reuse_pool_dir = os.path.join(disk, 'data', 'ReusedRecordsPool')
        task_id = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        task_dir = os.path.join(disk, 'data/smart_recorder', task_id)
        print('Recording bag to {}'.format(task_dir))
        log_dir = '/apollo/data/log'
        if not Path(log_dir).exists():
            os.makedirs(log_dir, exist_ok=True)
        log_file = '/apollo/data/log/smart_recorder.out'
        recorder_exe = '/apollo/bazel-bin/modules/data/tools/smart_recorder/smart_recorder'
        if not Path(recorder_exe).exists():
            flags_package_management = True
        recorder_exe = "/opt/apollo/neo/packages/apollo-data-dev/latest/bin/tools/smart_recorder/smart_recorder"
        if not Path(recorder_exe).exists():
            print("can't find smart_recorder! Have you installed apollo-data-dev package or built it?")
            exit(-1)
        if not flags_package_management:
            cmd = '''
                source /apollo/scripts/apollo_base.sh
                nohup {} --source_records_dir={} --restored_output_dir={} > {} 2>&1 &
            '''.format(recorder_exe, reuse_pool_dir, task_dir, log_file)
        else:
            cmd = '''
            nohup {} --source_records_dir={} --restored_output_dir={} > {} 2>&1 &
            '''.format(recorder_exe, reuse_pool_dir, task_dir, log_file)
        shell_cmd(cmd)

    @staticmethod
    def is_running():
        """Test if the given process running."""
        _, stdout, _ = shell_cmd('pgrep -f "smart_recorder" | grep -cv \'^1$\'', False)
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
