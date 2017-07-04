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

import subprocess
import glog

import config


def run_in_background(cmd, stdout_file, stderr_file):
    """Run command in background."""
    stdout_fd = open(config.Config.get_realpath(stdout_file), 'w')
    # Reuse the fd if it's the same file, such as the default '/dev/null'.
    stderr_fd = stdout_fd if stderr_file == stdout_file else open(
        config.Config.get_realpath(stderr_file), 'w')

    glog.info('Run command in background: {}'.format(cmd))
    subprocess.Popen(
        cmd, shell=True, stdout=stdout_fd, stderr=stderr_fd, close_fds=True)
