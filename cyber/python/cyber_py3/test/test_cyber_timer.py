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

"""Module for test cyber timer."""

import time
import unittest

from cyber_py3 import cyber
from cyber_py3 import cyber_timer


class TestCyberTimer(unittest.TestCase):
    """
    Class for cyber timer unit test.
    """

    count = 0

    @classmethod
    def setUpClass(cls):
        cyber.init()

    @classmethod
    def tearDownClass(cls):
        cyber.shutdown()

    def func(cls):
        print('Callback function is called [%d] times.' % cls.count)
        cls.count += 1

    def test_timer(self):
        ct = cyber_timer.Timer(100, self.func, 0)  # 100ms
        ct.start()
        time.sleep(1)  # 1s
        ct.stop()

        print('+' * 40 + 'test set_option' + '+' * 40)
        ct2 = cyber_timer.Timer()  # 10ms
        ct2.set_option(100, self.func, 0)
        ct2.start()
        time.sleep(1)  # 1s
        ct2.stop()


if __name__ == '__main__':
    unittest.main()
    # TODO(xiaoxq): It crashes here.
