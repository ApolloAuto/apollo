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

import importlib
import os
import sys


# init vars
CYBER_PATH = os.environ['CYBER_PATH']
CYBER_DIR = os.path.split(CYBER_PATH)[0]
sys.path.append(CYBER_PATH + "/third_party/")
sys.path.append(CYBER_PATH + "/lib/")

sys.path.append(CYBER_PATH + "/lib/python/")

sys.path.append(CYBER_DIR + "/python/")
sys.path.append(CYBER_DIR + "/cyber/")

_CYBER = importlib.import_module('_cyber_py3')
_CYBER_TIME = importlib.import_module('_cyber_time_py3')


class Duration(object):

    """
    Class for cyber Duration wrapper.
    """

    def __init__(self, other):
        if isinstance(other, int):
            self.nanoseconds_ = other
        elif isinstance(other, float):
            self.nanoseconds_ = other * 1000000000
        elif isinstance(other, Duration):
            self.nanoseconds_ = other.nanoseconds_
        self.duration_ = _CYBER_TIME.new_PyDuration(int(self.nanoseconds_))

    def __del__(self):
        _CYBER_TIME.delete_PyDuration(self.duration_)

    def sleep(self):
        """
        sleep for the amount of time specified by the duration.
        """
        _CYBER_TIME.PyDuration_sleep(self.duration_)

    def __str__(self):
        return str(self.nanoseconds_)

    def to_sec(self):
        """
        convert to second.
        """
        return float(self.nanoseconds_) / 1000000000

    def to_nsec(self):
        """
        convert to nanosecond.
        """
        return self.nanoseconds_

    def iszero(self):
        return self.nanoseconds_ == 0

    def __add__(self, other):
        return Duration(self.nanoseconds_ + other.nanoseconds_)

    def __radd__(self, other):
        return Duration(self.nanoseconds_ + other.nanoseconds_)

    def __sub__(self, other):
        return Duration(self.nanoseconds_ - other.nanoseconds_)

    def __lt__(self, other):
        return self.nanoseconds_ < other.nanoseconds_

    def __gt__(self, other):
        return self.nanoseconds_ > other.nanoseconds_

    def __le__(self, other):
        return self.nanoseconds_ <= other.nanoseconds_

    def __ge__(self, other):
        return self.nanoseconds_ >= other.nanoseconds_

    def __eq__(self, other):
        return self.nanoseconds_ == other.nanoseconds_

    def __ne__(self, other):
        return self.nanoseconds_ != other.nanoseconds_


class Time(object):

    """
    Class for cyber time wrapper.
    """

    ##
    # @brief Constructor, creates a Time.
    #
    # @param other float means seconds unit.
    # int means nanoseconds.
    def __init__(self, other):
        nanoseconds = 0
        if isinstance(other, int):
            nanoseconds = other
        elif isinstance(other, float):
            nanoseconds = other * 1000000000
        elif isinstance(other, Time):
            nanoseconds = other.to_nsec()

        self.time = _CYBER_TIME.new_PyTime(int(nanoseconds))

    def __del__(self):
        _CYBER_TIME.delete_PyTime(self.time)

    def __str__(self):
        return str(self.to_nsec())

    def iszero(self):
        return self.to_nsec() == 0

    @staticmethod
    def now():
        """
        return current time.
        """
        # print _CYBER_TIME.PyTime_now()
        # print type(_CYBER_TIME.PyTime_now())
        time_now = Time(_CYBER_TIME.PyTime_now())
        return time_now

    @staticmethod
    def mono_time():
        mono_time = Time(_CYBER_TIME.PyTime_mono_time())
        return mono_time

    def to_sec(self):
        """
        convert to second.
        """
        return _CYBER_TIME.PyTime_to_sec(self.time)

    def to_nsec(self):
        """
        convert to nanosecond.
        """
        return _CYBER_TIME.PyTime_to_nsec(self.time)

    def sleep_until(self, cyber_time):
        """
        sleep until time.
        """
        if isinstance(time, Time):
            return _CYBER_TIME.PyTime_sleep_until(self.time,
                                                  cyber_time.to_nsec())
        return NotImplemented

    def __sub__(self, other):
        if isinstance(other, Time):
            return Duration(self.to_nsec() - other.to_nsec())
        else:
            isinstance(other, Duration)
            return Time(self.to_nsec() - other.to_nsec())

    def __add__(self, other):
        return Time(self.to_nsec() + other.to_nsec())

    def __radd__(self, other):
        return Time(self.to_nsec() + other.to_nsec())

    def __lt__(self, other):
        return self.to_nsec() < other.to_nsec()

    def __gt__(self, other):
        return self.to_nsec() > other.to_nsec()

    def __le__(self, other):
        return self.to_nsec() <= other.to_nsec()

    def __ge__(self, other):
        return self.to_nsec() >= other.to_nsec()

    def __eq__(self, other):
        return self.to_nsec() == other.to_nsec()

    def __ne__(self, other):
        return self.to_nsec() != other.to_nsec()


class Rate(object):

    """
    Class for cyber Rate wrapper. Help run loops at a desired frequency.
    """

    ##
    # @brief Constructor, creates a Rate.
    #
    # @param other float means frequency the desired rate to run at in Hz.
    # int means the expected_cycle_time.
    def __init__(self, other):
        if isinstance(other, int):
            self.rate_ = _CYBER_TIME.new_PyRate(other)
        elif isinstance(other, float):
            self.rate_ = _CYBER_TIME.new_PyRate(int(1.0 / other))
        elif isinstance(other, Duration):
            self.rate_ = _CYBER_TIME.new_PyRate(other.to_nsec())

    def __del__(self):
        _CYBER_TIME.delete_PyRate(self.rate_)

    def __str__(self):
        return "cycle_time = %s, exp_cycle_time = %s" % (str(self.get_cycle_time()), str(self.get_expected_cycle_time()))

    def sleep(self):
        """
        Sleeps for any leftover time in a cycle.
        """
        _CYBER_TIME.PyRate_sleep(self.rate_)

    def reset(self):
        """
        Sets the start time for the rate to now.
        """
        _CYBER_TIME.PyRate_PyRate_reset(self.rate_)

    def get_cycle_time(self):
        """
        Get the actual run time of a cycle from start to sleep.
        """
        return Duration(_CYBER_TIME.PyRate_get_cycle_time(self.rate_))

    def get_expected_cycle_time(self):
        """
        Get the expected cycle time.
        """
        return Duration(_CYBER_TIME.PyRate_get_expected_cycle_time(self.rate_))
