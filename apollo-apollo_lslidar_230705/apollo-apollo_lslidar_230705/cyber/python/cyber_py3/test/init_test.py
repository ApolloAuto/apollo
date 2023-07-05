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
"""Module for test init."""

import unittest

from cyber.python.cyber_py3 import cyber

class TestInit(unittest.TestCase):

    """
    Class for init unit test.
    """

    def test_init(self):
        """
        Test cyber.
        """
        self.assertTrue(cyber.init())
        self.assertTrue(cyber.ok())
        cyber.shutdown()
        self.assertTrue(cyber.is_shutdown())


if __name__ == '__main__':
    unittest.main()
