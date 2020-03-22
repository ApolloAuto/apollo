#!/usr/bin/env python2

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

import ctypes
import importlib
import os
import sys


PY_TIMER_CB_TYPE = ctypes.CFUNCTYPE(ctypes.c_void_p)

# init vars
CYBER_PATH = os.environ.get('CYBER_PATH', '/apollo/cyber')
CYBER_DIR = os.path.split(CYBER_PATH)[0]
sys.path.append(CYBER_PATH + "/third_party/")
sys.path.append(CYBER_PATH + "/lib/")

sys.path.append(CYBER_PATH + "/lib/python/")

sys.path.append(CYBER_DIR + "/python/")
sys.path.append(CYBER_DIR + "/cyber/")

_CYBER_TIMER = importlib.import_module('_cyber_timer')


class Timer(object):

    """
    Class for cyber timer wrapper.
    """

    ##
    # @brief Used to perform oneshot or periodic timing tasks
    #
    # @param period The period of the timer, unit is ms.
    # @param callback The tasks that the timer needs to perform.
    # @param oneshot 1:perform the callback only after the first timing cycle
    # 0:perform the callback every timed period
    def __init__(self, period=None, callback=None, oneshot=None):
        if period is None and callback is None and oneshot is None:
            self.timer = _CYBER_TIMER.new_PyTimer_noparam()
        else:
            self.timer_cb = PY_TIMER_CB_TYPE(callback)
            self.f_ptr_cb = ctypes.cast(self.timer_cb, ctypes.c_void_p).value
            self.timer = _CYBER_TIMER.new_PyTimer(
                period, self.f_ptr_cb, oneshot)

    def __del__(self):
        _CYBER_TIMER.delete_PyTimer(self.timer)

    ##
    # @brief set the option of timer.
    #
    # @param period The period of the timer, unit is ms.
    # @param callback The tasks that the timer needs to perform.
    # @param oneshot 1:perform the callback only after the first timing cycle
    # 0:perform the callback every timed period
    def set_option(self, period, callback, oneshot=0):
        self.timer_cb = PY_TIMER_CB_TYPE(callback)
        self.f_ptr_cb = ctypes.cast(self.timer_cb, ctypes.c_void_p).value
        _CYBER_TIMER.PyTimer_set_option(
            self.timer, period, self.f_ptr_cb, oneshot)

    ##
    # @brief start the timer
    def start(self):
        _CYBER_TIMER.PyTimer_start(self.timer)

    ##
    # @brief stop the timer
    def stop(self):
        _CYBER_TIMER.PyTimer_stop(self.timer)
