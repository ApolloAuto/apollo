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
"""Module for example of cyber time."""

import time

from cyber.python.cyber_py3 import cyber_time


def test_time():
    ct = cyber_time.Time(123)
    print(ct.to_nsec())
    ftime = ct.now().to_sec()
    print(ftime)
    time.sleep(1)
    ftime = cyber_time.Time.now().to_sec()
    print(ftime)
    ftime = cyber_time.Time.mono_time().to_sec()
    print(ftime)

    td1 = cyber_time.Duration(111)
    tm1 = ct - td1
    print("ct sub du is ", tm1)
    tm1 = ct - td1
    print(tm1)

    tm5 = cyber_time.Time(1.8)
    tm7 = cyber_time.Time(tm5)
    print(tm7)


def test_duration():
    td1 = cyber_time.Duration(111)
    td2 = cyber_time.Duration(601000000000)
    td3 = td2 - td1
    print(td1, td1.to_nsec())
    print(td2, td2.to_nsec())
    print(td3, td3.to_nsec())
    print(td2.to_sec())
    print(td2.iszero())
    print(str(td2))
    td5 = cyber_time.Duration(1.8)
    td6 = td5
    print(type(td6))
    td7 = cyber_time.Duration(td6)
    print(td7)
    # td7 = cyber_time.Duration("zzz")
    # print(td7)
    # print(type(td2))


def test_rate():
    rt1 = cyber_time.Rate(111)
    rt2 = cyber_time.Rate(0.2)
    rt3 = cyber_time.Rate(cyber_time.Duration(666))
    print(rt1)
    print(rt2)
    print(rt3)


if __name__ == '__main__':
    print("test time", "-" * 50)
    test_time()
    print("test duration", "-" * 50)
    test_duration()
    print("test rate", "-" * 50)
    test_rate()
