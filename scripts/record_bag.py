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

1. If we have NVME disk, it will be used to record all data.
2. If we have Non-NVME disk, it will only record smaller topics (blacklist LARGE_TOPICS), unless '--all' is
   specified.
3. If no external disks are available, we will take '/apollo' as a
   'Non-NVME disk' and follow the rule above.

Run with '--help' to see more options.
"""

import argparse
import datetime
import os
import subprocess
import sys
import shutil

import psutil

APOLLO_ENV_ROOT = os.getenv("APOLLO_ENV_WORKROOT") or "/apollo"
APOLLO_RUNTIME_PATH = os.getenv("APOLLO_RUNTIME_PATH") or "/apollo"

LARGE_TOPICS = [
    '/apollo/sensor/camera/front_12mm/image',
    '/apollo/sensor/camera/front_12mm/image/compressed',
    '/apollo/sensor/camera/front_12mm/video/compressed',
    '/apollo/sensor/camera/front_6mm/image',
    '/apollo/sensor/camera/front_6mm/image/compressed',
    '/apollo/sensor/camera/front_6mm/video/compressed',
    '/apollo/sensor/camera/left_fisheye/image',
    '/apollo/sensor/camera/left_fisheye/image/compressed',
    '/apollo/sensor/camera/left_fisheye/video/compressed',
    '/apollo/sensor/camera/rear_6mm/image',
    '/apollo/sensor/camera/rear_6mm/image/compressed',
    '/apollo/sensor/camera/rear_6mm/video/compressed',
    '/apollo/sensor/camera/right_fisheye/image',
    '/apollo/sensor/camera/right_fisheye/image/compressed',
    '/apollo/sensor/camera/right_fisheye/video/compressed',
    '/apollo/sensor/lidar128/PointCloud2',
    '/apollo/sensor/lidar128/compensator/PointCloud2',
    '/apollo/sensor/lidar16/PointCloud2',
    '/apollo/sensor/lidar16/Scan',
    '/apollo/sensor/lidar16/compensator/PointCloud2',
    '/apollo/sensor/lidar16/front/center/PointCloud2',
    '/apollo/sensor/lidar16/front/up/PointCloud2',
    '/apollo/sensor/lidar16/fusion/PointCloud2',
    '/apollo/sensor/lidar16/fusion/compensator/PointCloud2',
    '/apollo/sensor/lidar16/rear/left/PointCloud2',
    '/apollo/sensor/lidar16/rear/right/PointCloud2',
    '/apollo/sensor/microphone',
    '/apollo/sensor/radar/front',
    '/apollo/sensor/radar/rear',
    '/apollo/sensor/velodyne64/compensator/PointCloud2',
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
        self.parser.add_argument('--stop_signal', default="SIGINT",
                                 help='Signal to stop the recorder.')
        self.parser.add_argument('--all', default=False, action="store_true",
                                 help='Record all topics even without high '
                                 'performance disks.')
        self.parser.add_argument('--small', default=False, action="store_true",
                                 help='Record small topics only.')
        self.parser.add_argument('--split_duration', default="1m",
                                 help='Duration to split bags, will be applied '
                                 'as parameter to "rosbag record --duration".')
        self.parser.add_argument('--default_name', default="",
                                 help='Change the default name of the record')
        self.parser.add_argument('--rename', default="",
                                 help='Change the new name of the record')
        self.parser.add_argument('--dreamview', default=False, action="store_true",
                                 help='Path for dreamview fixed recording file.')
        self.parser.add_argument('--delete', default=False, action="store_true",
                                 help='Delete record.')
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
            available_size = DiskManager.disk_avail_size(disk.mountpoint)
            if available_size <= 0:
                continue
            disks.append({
                'mountpoint': disk.mountpoint,
                'available_size': available_size,
                'is_nvme': disk.mountpoint.startswith('/media/apollo/internal_nvme'),
            })
        # Prefer NVME disks and then larger disks.
        self.disks = sorted(
            disks, reverse=True,
            key=lambda disk: (disk['is_nvme'], disk['available_size']))

    @staticmethod
    def disk_avail_size(disk_path):
        """Get disk available size."""
        try:
            statvfs = os.statvfs(disk_path)
            return statvfs.f_frsize * statvfs.f_bavail
        except Exception as e:
            return 0


class Recorder(object):
    """Data recorder."""

    def __init__(self, args):
        self.args = args
        self.disk_manager = DiskManager()
        self.record_path_file = os.path.join(
            APOLLO_ENV_ROOT, 'data/bag', 'record_path_file')
        self.dreamview_record_path = os.path.join(
            os.path.expanduser("~"), '.apollo/resources/records')

    def start(self):
        """Start recording."""
        if Recorder.is_running():
            print('Another data recorder is running, skip.')
            return

        disks = self.disk_manager.disks
        # To record all topics if
        # 1. User requested with '--all' argument.
        # 2. Or we have a NVME disk and '--small' is not set.
        record_all = self.args.all or (
            len(disks) > 0 and disks[0]['is_nvme'] and not self.args.small)
        # Use the best disk, or fallback '/apollo' if none available.
        disk_to_use = disks[0]['mountpoint'] if len(
            disks) > 0 else APOLLO_ENV_ROOT

        # If the start command comes from dreamview, save to the default path.
        if self.args.dreamview:
            disk_to_use = self.dreamview_record_path

        # Record small topics to quickly copy and process
        if record_all:
            self.record_task(disk_to_use, False)

        self.record_task(disk_to_use, record_all)

    def stop(self):
        """Stop recording."""
        shell_cmd('pkill --signal {} -f "cyber_recorder record"'.format(
            self.args.stop_signal))

    def record_task(self, disk, record_all):
        """Record tasks into the <disk>/data/bag/<task_id> directory."""
        task_id = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        # If a default name is set.
        if self.args.default_name != "":
            task_id = self.args.default_name
        if not record_all:
            task_id += "_s"

        task_dir = ''
        if self.args.dreamview:
            task_dir = os.path.join(disk, task_id)
        else:
            task_dir = os.path.join(disk, 'data/bag', task_id)
        print('Recording bag to {}'.format(task_dir))

        topics_str = "--all"

        log_file = APOLLO_ENV_ROOT + '/data/log/apollo_record.out'
        if not record_all:
            log_file += "_s"
            topics_str += " -k {}".format(' -k '.join(list(LARGE_TOPICS)))

        if not os.path.exists(task_dir):
            os.makedirs(task_dir)
        cmd = '''
            cd "{task_dir}"
            source {apollo_runtime_path}/scripts/apollo_base.sh
            nohup cyber_recorder record {topics_str} >{log_file} 2>&1 &
        '''.format(task_dir=task_dir, apollo_runtime_path=APOLLO_RUNTIME_PATH,
                   topics_str=topics_str, log_file=log_file)
        shell_cmd(cmd)

    def file_exist_check(self, task_dir, old_paths, new_paths, task_dir_flag=False):
        """Check if the file exists"""
        if not os.path.exists(task_dir):
            print('The dreamview recording package does not exist, please record through dreamview.')
            exit(-1)
        for filename in os.listdir(task_dir):
            if "record" in filename:
                # Extract the part of the filename before ".record".
                name_parts = filename.split(".record")
                if len(name_parts) > 1:
                    extension = name_parts[1] + ".record"

                    # Construct the new filename.
                    new_filename = ''
                    if task_dir_flag:
                        new_filename = f"{self.args.rename}_s{extension}"
                    else:
                        new_filename = f"{self.args.rename}{extension}"
                    old_path = os.path.join(task_dir, filename)
                    new_path = os.path.join(
                        os.path.dirname(task_dir), new_filename)
                    if os.path.exists(new_path):
                        print(f"The file '{new_path}' already exist.")
                        exit(-1)
                    old_paths.append(old_path)
                    new_paths.append(new_path)

    def rename_files_in_directory(self, old_paths, new_paths):
        """rename files"""
        for i in range(len(old_paths)):
            shutil.move(old_paths[i], new_paths[i])
            print(f"Move '{old_paths[i]}' to '{new_paths[i]}'")
        self.delete()

    def rename(self):
        """Saving is actually modifying the name of the record just recorded."""
        task_dir = os.path.join(
            self.dreamview_record_path, self.args.default_name)
        task_s_dir = task_dir + '_s'
        old_paths, new_paths = [], []
        self.file_exist_check(task_dir, old_paths, new_paths)
        self.file_exist_check(task_s_dir, old_paths, new_paths, True)
        self.rename_files_in_directory(old_paths, new_paths)

        # print('save')
    def delete(self):
        """Delete the file just recorded."""
        task_dir = os.path.join(
            self.dreamview_record_path, self.args.default_name)
        task_s_dir = task_dir + '_s'
        shutil.rmtree(task_dir)
        shutil.rmtree(task_s_dir)

    @staticmethod
    def is_running():
        """Test if the given process running."""
        _, stdout, _ = shell_cmd(
            'pgrep -f "cyber_recorder record" | grep -cv \'^1$\'', False)
        # If stdout is the pgrep command itself, no such process is running.
        return stdout.strip() != '1' if stdout else False


def main():
    """Main entry."""
    arg_manager = ArgManager()
    args = arg_manager.args()
    recorder = Recorder(args)
    if args.stop:
        recorder.stop()
    elif args.start:
        recorder.start()
    elif args.delete:
        recorder.delete()
    else:
        recorder.rename()


if __name__ == '__main__':
    main()
