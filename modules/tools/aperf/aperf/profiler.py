#!/usr/bin/env python3

###############################################################################
# Copyright 2022 The Apollo Authors. All Rights Reserved.
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

from aperf.cost import FuncCost, TaskCost, TotalCost


class Profiler():
    def __init__(self) -> None:
        """Parse perf class
        """
        self.total_cost = TotalCost()

    def parse(self, log_file):
        """Parse perf from log file

        Args:
            log_file (_type_): perf log file
        """
        with open(log_file, 'r') as f:
            task_name, task_cost = None, None
            for line in f.readlines()[3:]:
                if self._is_frame_start(line):
                    if task_name:
                        self.total_cost.add(task_cost)
                    task_name = self._get_frame_name(line)
                    task_cost = TaskCost(task_name)
                else:
                    func_cost = FuncCost()
                    # Use "]" to split
                    func_cost.parse(self._get_valid_data(line))
                    task_cost.add(func_cost)
            if task_name:
                self.total_cost.add(task_cost)

    def draw_timecost(self, task_name):
        """Draw the time cost of task_name

        Args:
            task_name (str): task cost class name
        """
        if task_name:
            self.total_cost.draw_task_cost(task_name)
        else:
            self.total_cost.draw_all_cost()

    def draw_timeline(self):
        """Draw the timeline

        Args:
            task_name (str): task cost class name
        """
        self.total_cost.draw_timeline()

    def _is_frame_start(self, line):
        """Whether it is the start of frame

        Args:
            line (str): line in log file

        Returns:
            bool: whether it is the start of frame
        """
        return "Frame :" in self._get_valid_data(line)

    def _get_valid_data(self, line):
        """Get valid data by removing invalid headers

        Args:
            line (str): line in log file

        Returns:
            str: valid data
        """
        data = line.split(']')
        return data[1] if len(data) > 1 else ""

    def _get_frame_name(self, line):
        """Get frame name

        Args:
            line (str): line in log file

        Returns:
            str: frame name
        """
        valid_data = self._get_valid_data(line)
        return valid_data.split(':')[1].strip()
