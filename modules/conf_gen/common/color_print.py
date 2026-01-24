#!/usr/bin/env python3

###############################################################################
# Copyright 2024 The Apollo Authors. All Rights Reserved.
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
Print colorful text
"""


class Bcolors:
    """ Color Definition
    """
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def info(msg):
    """ info print
    """
    print(Bcolors.OKBLUE + 'Info: ' + Bcolors.ENDC + msg)


def warn(msg):
    """ info print
    """
    print(Bcolors.WARNING + 'Warn: ' + msg + Bcolors.ENDC)


def error(msg):
    """ info print
    """
    print(Bcolors.FAIL + 'ERROR: ' + msg + Bcolors.ENDC)


def succ(msg):
    """ info print
    """
    print(Bcolors.OKGREEN + msg + Bcolors.ENDC)
