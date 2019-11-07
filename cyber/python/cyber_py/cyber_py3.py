#!/usr/bin/env python3

# ****************************************************************************
# Copyright 2019 The Apollo Authors. All Rights Reserved.
#
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
sys.path = [path for path in sys.path if "python2.7" not in path]

import os
import importlib
import time
import threading
import ctypes

PY_CALLBACK_TYPE = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p)
PY_CALLBACK_TYPE_T = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p)

# init vars
CYBER_PATH = os.environ['CYBER_PATH']
CYBER_DIR = os.path.split(CYBER_PATH)[0]
sys.path.append(CYBER_PATH + "/third_party/")
sys.path.append(CYBER_PATH + "/lib/")
sys.path.append(CYBER_PATH + "/python/cyber")
sys.path.append(CYBER_PATH + "/python/cyber_py")

sys.path.append(CYBER_PATH + "/lib/python/")

sys.path.append(CYBER_DIR + "/python/")
sys.path.append(CYBER_DIR + "/cyber/")

_CYBER_INIT = importlib.import_module('_cyber_init_py3')


##
# @brief init cyber environment.
# @param module_name Used as the log file name.
#
# @return Success is True, otherwise False.
def init(module_name="cyber_py"):
    """
    init cyber environment.
    """
    return _CYBER_INIT.py_init(module_name)


def ok():
    """
    is cyber envi ok.
    """
    return _CYBER_INIT.py_ok()


def shutdown():
    """
    shutdown cyber envi.
    """
    return _CYBER_INIT.py_shutdown()


def is_shutdown():
    """
    is cyber shutdown.
    """
    return _CYBER_INIT.py_is_shutdown()


def waitforshutdown():
    """
    wait until the cyber is shutdown.
    """
    return _CYBER_INIT.py_waitforshutdown()
