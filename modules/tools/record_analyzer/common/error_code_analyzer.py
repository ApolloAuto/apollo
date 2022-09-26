#!/usr/bin/env python3

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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

from modules.common_msgs.basic_msgs import error_code_pb2
from modules.tools.record_analyzer.common.statistical_analyzer import PrintColors
from modules.tools.record_analyzer.common.distribution_analyzer import DistributionAnalyzer


class ErrorCodeAnalyzer:
    """class"""

    def __init__(self):
        """init"""
        self.error_code_count = {}

    def put(self, error_code):
        """put"""
        error_code_name = \
            error_code_pb2.ErrorCode.Name(error_code)
        if error_code_name not in self.error_code_count:
            self.error_code_count[error_code_name] = 1
        else:
            self.error_code_count[error_code_name] += 1

    def print_results(self):
        """print"""
        DistributionAnalyzer().print_distribution_results(self.error_code_count)
