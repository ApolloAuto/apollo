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

"""Module for example of timer."""

import time

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_timer


count = 0


def fun():
    global count
    print("cb fun is called:", count)
    count += 1


def test_timer():
    cyber.init()
    ct = cyber_timer.Timer(10, fun, 0)  # 10ms
    ct.start()
    time.sleep(1)  # 1s
    ct.stop()

    print("+" * 80, "test set_option")
    ct2 = cyber_timer.Timer()  # 10ms
    ct2.set_option(10, fun, 0)
    ct2.start()
    time.sleep(1)  # 1s
    ct2.stop()

    cyber.shutdown()


if __name__ == '__main__':
    test_timer()
