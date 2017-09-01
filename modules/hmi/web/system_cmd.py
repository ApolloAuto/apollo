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
"""System command utils."""

import shutil
import subprocess

from config import Config


def async_run_command(cmd, stdout_file, stderr_file):
    """Run command in background."""
    stdout_fd = open(Config.get_realpath(stdout_file), 'w')
    # Reuse the fd if it's the same file, such as the default '/dev/null'.
    stderr_fd = stdout_fd if stderr_file == stdout_file else open(
        Config.get_realpath(stderr_file), 'w')

    Config.log.info('Run command in background: {}'.format(cmd))
    nohup_cmd = 'nohup {} &'.format(cmd)
    subprocess.Popen(nohup_cmd, shell=True,
                     stdout=stdout_fd, stderr=stderr_fd, close_fds=True
                     ).wait()


def async_run_command_pb(cmd_pb):
    """Run an apollo.hmi.Command in background."""
    # Construct the command string by joining all components.
    cmd_pb.command[0] = Config.get_realpath(cmd_pb.command[0])
    cmd_str = ' '.join(cmd_pb.command)
    async_run_command(cmd_str, cmd_pb.stdout_file, cmd_pb.stderr_file)


def copy(src, dst):
    """Copy file from src to dst if they are not the same."""
    if src != dst:
        shutil.copy(Config.get_realpath(src), Config.get_realpath(dst))


def copytree(src, dst):
    """Copy directory, clear the dst if it existed."""
    dst = Config.get_realpath(dst)
    shutil.rmtree(dst, ignore_errors=True)
    shutil.copytree(Config.get_realpath(src), dst)
