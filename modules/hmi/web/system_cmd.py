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

import os
import shutil
import subprocess

from config import Config


def async_run_command(cmd, stdout_file, stderr_file):
    """Run command in background."""
    stdout_fd = open(stdout_file, 'w')
    # Reuse the fd if it's the same file, such as the default '/dev/null'.
    stderr_fd = stdout_fd if stderr_file == stdout_file else open(
        stderr_file, 'w')

    Config.log.info('Run command in background: {}'.format(cmd))
    nohup_cmd = 'nohup {} &'.format(cmd)
    subprocess.Popen(nohup_cmd, shell=True,
                     stdout=stdout_fd, stderr=stderr_fd, close_fds=True
                     ).wait()


def async_run_command_pb(cmd_pb):
    """Run an apollo.hmi.Command in background."""
    # Construct the command string by joining all components.
    cmd_str = ' '.join(cmd_pb.command)
    async_run_command(cmd_str, cmd_pb.stdout_file, cmd_pb.stderr_file)


def safe_copy(src, dst):
    """Safely copy from src to dst if they are not the same."""
    if src == dst:
        Config.log.debug('Skip copying same path %s', src)
        return
    if os.path.isdir(src):
        Config.log.debug('Copying directory from %s to %s', src, dst)
        shutil.rmtree(dst, ignore_errors=True)
        shutil.copytree(src, dst)
    elif os.path.isfile(src):
        Config.log.debug('Copying file from %s to %s', src, dst)
        shutil.copyfile(src, dst)
