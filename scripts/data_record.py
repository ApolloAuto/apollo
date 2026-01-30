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
Start apollo data recorder.

Usage:
    1. record all channels
    > python data_record.py --all

    2. record channels exclude image channels
    // exclude all pointcloud channel
    > python data_record.py --middle 
    
    // include compensator channel
    > python data_record.py --middle --with_compensator

    3. record channels exclude all lidar and image channels
    > python data_record.py --small

    4. stop record
    > python data_record.py --stop
"""
import argparse
import datetime
import os
import subprocess
import sys

APOLLO_ENV_ROOT = os.getenv("APOLLO_ENV_WORKROOT") or "/apollo"
default_record_dir = os.path.join(APOLLO_ENV_ROOT, 'data/record')
record_args_dict = {
    "all": "-a",
    "middle": "-a -k '/apollo/.*/image' -k '/apollo/.*/PointCloud2'",
    "middle_with_compensator": 
        "-a  -k '^(?!.*compensator)(?!.*back).*/PointCloud2' -k /apollo/.*/Scan -k '/apollo/.*/image'",
    "small": "-a -k '/apollo/.*/PointCloud2' -k '/apollo/.*/image'"
}


def shell_cmd(cmd, alert_on_failure=True):
    """ Execute shell command and return (ret-code, stdout, stderr)
    """
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
    """ Arguments manager.
    """

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
                                 help='Record all channels')
        self.parser.add_argument('--middle', default=False, action="store_true",
                                 help='record channels exclude lidar and image channels')
        self.parser.add_argument('--with_compensator', default=False, action="store_true",
                                 help='record middle channels but include compensator channel')
        self.parser.add_argument('--small', default=False, action="store_true",
                                 help='record channels exclude lidar and image channels')
        self.parser.add_argument('--foreground', default=False, action="store_true",
                                 help='run cyber_recorder record as a foreground process')
        self.parser.add_argument('-d', '--dest_dir', default=default_record_dir,
                                 help='target record dirname')
        self._args = None

    def args(self):
        """ Get parsed args.
        """
        if self._args is None:
            self._args = self.parser.parse_args()
        return self._args


class Recorder(object):
    """ Data recorder
    """

    def __init__(self, args):
        self.args = args

    def start(self):
        """ Start recording.
        """
        if Recorder.is_running():
            print('Another data recorder is running, skip.')
            return
        has_record_task = False
        if self.args.all:
            self.record_task('all', record_args_dict['all'])
            has_record_task = True
        if self.args.middle:
            print(self.args.with_compensator)
            if self.args.with_compensator:
                self.record_task('middle',
                                 record_args_dict['middle_with_compensator'])
            else:
                self.record_task('middle', record_args_dict['middle'])
            has_record_task = True
        if self.args.small:
            self.record_task('small', record_args_dict['small'])
            has_record_task = True
        # default: record all
        if not has_record_task:
            self.record_task('all', record_args_dict['all'])

    def stop(self):
        """ Stop recording.
        """
        shell_cmd('pkill --signal {} -f "cyber_recorder record"'.format(
            self.args.stop_signal))

    def record_task(self, mode, record_args):
        """ Record task
        """
        task_id = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S') + "_" + mode
        task_dir = os.path.join(self.args.dest_dir, task_id)
        print('Recording to {}'.format(task_dir))
        if not os.path.exists(task_dir):
            os.makedirs(task_dir)
        log_file = f'{APOLLO_ENV_ROOT}/data/log/apollo_record_{mode}.out'
        cmd = '''cd "{task_dir}"
                 nohup cyber_recorder record {record_args} >{log_file} 2>&1'''.format(
            task_dir=task_dir, record_args=record_args, log_file=log_file)
        if not self.args.foreground:
            cmd += " &"
        shell_cmd(cmd)

    @staticmethod
    def is_running():
        """ Test if the given process running.
        """
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
    else:
        recorder.start()


if __name__ == '__main__':
    main()
