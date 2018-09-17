# ****************************************************************************
# Copyright 2018 The Apollo Authors. All Rights Reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ****************************************************************************
# -*- coding: utf-8 -*-
"""Module for init environment."""

import sys
import os
import importlib

# init vars
CYBERTRON_PATH = os.environ['CYBERTRON_PATH']
CYERTRON_DIR = os.path.split(CYBERTRON_PATH)[0]
sys.path.append(CYBERTRON_PATH + "/third_party/")
sys.path.append(CYBERTRON_PATH + "/lib/")
sys.path.append(CYBERTRON_PATH + "/python/cybertron")

sys.path.append(CYERTRON_DIR + "/python/")
sys.path.append(CYERTRON_DIR + "/cybertron/")

_CYBER_INIT = importlib.import_module('_cyber_init')

def init():
    """
    init cybertron.
    """
    return _CYBER_INIT.py_init()

def ok():
    """
    is cybertron envi ok.
    """
    return _CYBER_INIT.py_ok()

def shutdown():
    """
    shutdown cybertron envi.
    """
    return _CYBER_INIT.py_shutdown()

def is_shutdown():
    """
    is cybertron shutdown.
    """
    return _CYBER_INIT.py_is_shutdown()

def waitforshutdown():
    """
    waitforshutdown.
    """
    return _CYBER_INIT.py_waitforshutdown()
