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
"""Module for test cyber_time."""

import time
import unittest

from cyber_py import cyber
from cyber_py import cyber_time


class TestTime(unittest.TestCase):

    """
    Class for node unit test.
    """
    @classmethod
    def setUpClass(cls):
        cyber.init()

    @classmethod
    def tearDownClass(cls):
        cyber.shutdown()

    def test_time(self):
        ct = cyber_time.Time(123)
        self.assertEqual(123, ct.to_nsec())
        ftime = ct.now().to_sec()
        print(ftime)
        time.sleep(1)
        ftime = cyber_time.Time.now().to_sec()
        print(ftime)
        ftime = cyber_time.Time.mono_time().to_sec()
        print(ftime)

        td1 = cyber_time.Duration(111)
        tm1 = ct - td1
        self.assertEqual(12, tm1.to_nsec())

        tm5 = cyber_time.Time(1.8)
        self.assertFalse(tm5.iszero())
        self.assertEqual(1800000000, tm5.to_nsec())
        tm7 = cyber_time.Time(tm5)
        self.assertEqual(1800000000, tm7.to_nsec())

    def test_duration(self):
        td1 = cyber_time.Duration(111)
        td2 = cyber_time.Duration(601000000000)
        td3 = td2 - td1
        self.assertEqual(111, td1.to_nsec())
        self.assertEqual(601000000000, td2.to_nsec())
        self.assertEqual(601000000000 - 111, td3.to_nsec())

        print(td2.to_sec())
        self.assertEqual(601.0, td2.to_sec())
        self.assertFalse(td2.iszero())
        print(str(td2))
        td5 = cyber_time.Duration(1.8)
        td6 = td5
        print(type(td6))
        self.assertTrue(isinstance(td6, cyber_time.Duration))

    def test_rate(self):
        rt1 = cyber_time.Rate(111)
        self.assertEqual(0, rt1.get_cycle_time().to_nsec())
        self.assertEqual(111, rt1.get_expected_cycle_time().to_nsec())
        rt2 = cyber_time.Rate(0.2)
        self.assertEqual(0, rt2.get_cycle_time().to_nsec())
        self.assertEqual(5, rt2.get_expected_cycle_time().to_nsec())
        rt3 = cyber_time.Rate(cyber_time.Duration(666))
        self.assertEqual(0, rt3.get_cycle_time().to_nsec())
        self.assertEqual(666, rt3.get_expected_cycle_time().to_nsec())

if __name__ == '__main__':
    unittest.main()
