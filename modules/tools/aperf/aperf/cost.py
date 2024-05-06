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

import logging
import matplotlib.pyplot as plt
from matplotlib.widgets import Cursor
import statistics

from collections import defaultdict


class FuncCost():
    def __init__(self, name="", level=0, duration=0, start_time=0,
                 end_time=0) -> None:
        """Function cost class

        Args:
            name (str, optional): function name. Defaults to "".
            level (int, optional): function nesting level. Defaults to 0.
            duration (int, optional): function time cost. Defaults to 0.
            start_time (int, optional): function start time. Defaults to 0.
            end_time (int, optional): function end time. Defaults to 0.
        """
        self.name = name
        self.level = level
        self.duration = duration
        self.start_time = start_time
        self.end_time = end_time

    def parse(self, line):
        """parse function cost class for line

        Args:
            line (str): line in perf.INFO contain function cost
        """
        data = line.strip().split(',')
        if len(data) < 5:
            return
        self.level = int(data[0])
        self.name = data[1]
        self.duration = int(data[2])
        self.start_time = int(data[3])
        self.end_time = int(data[4])


class TaskCost():
    def __init__(self, name) -> None:
        """Task cost class (a component or module)

        Args:
            name (str): Task cost name
        """
        self.name = name
        self.func_costs = defaultdict()

    def add(self, func_cost):
        """Add func cost to task cost

        Args:
            func_cost (FuncCost): function cost class
        """
        if func_cost.name in self.func_costs:
            print("Duplicate name {}".format(func_cost.name))
        self.func_costs[func_cost.name] = func_cost

    def get_func_cost(self, func_name):
        """Get func cost

        Args:
            func_name (str): function cost name

        Returns:
            FuncCost: function cost class
        """
        return self.func_costs.get(func_name, None)

    def total(self):
        """total task time cost in milliseconds

        Returns:
            float: task total cost
        """
        total_cost = 0.0
        for _, func_cost in self.func_costs.items():
            if func_cost.level == 1:
                total_cost += func_cost.duration
        return total_cost / 1e6

    def start_and_end(self):
        """task start and end time

        Returns:
            _type_: start and end time
        """
        start_time, end_time = 0, 0
        for _, func_cost in self.func_costs.items():
            if func_cost.level == 1:
                start_time, end_time = func_cost.start_time, func_cost.end_time
        return start_time, end_time

    def max(self):
        """multi-node tree to find leaf node, find max time cost node
        """
        pass

    def min(self):
        """multi-node tree to find leaf node, find min time cost node
        """
        pass

    def max_k(self):
        """multi-node tree to find leaf node, find max k time cost node
        """
        pass

    def min_k(self):
        """multi-node tree to find leaf node, find min k time cost node
        """
        pass

    def large_than(self, threshold):
        """multi-node tree to find leaf node, find time cost node large than
           threshold
        """
        pass


class TotalCost():
    def __init__(self) -> None:
        """Total cost class
        """
        self.task_cost_dict = defaultdict(list)

    def add(self, task_cost):
        """Add task cost to total cost

        Args:
            task_cost (TaskCost): task cost class
        """
        self.task_cost_dict[task_cost.name].append(task_cost)

    def draw_all_cost(self):
        """Draw the time cost of all tasks
        """
        for task_name, task_costs in self.task_cost_dict.items():
            costs = [task_cost.total() for task_cost in task_costs]
            plt.plot(costs, label=task_name)
        plt.ylabel("Time (ms)")
        plt.legend()
        plt.show()

    def draw_timeline(self):
        """Draw the time line of all tasks
        """
        y, task_names = 0, [""]
        fig, ax = plt.subplots()
        for task_name, task_costs in self.task_cost_dict.items():
            for task_cost in task_costs:
                ax.plot(task_cost.start_and_end(), [y, y])
            y += 1
            task_names.append(task_name)
        ax.set_yticklabels(task_names)
        cursor = Cursor(ax, color='r', linewidth=1, horizOn=False)
        plt.show()

    def draw_task_cost(self, task_name):
        """Draw the time cost of task_name
        """
        task_costs = self.task_cost_dict.get(task_name, None)
        if task_costs:
            costs = [task_cost.total() for task_cost in task_costs]
            logging.debug(costs)
            mean, std = self.task_cost_mean(
                task_name), self.task_cost_std(task_name)
            fig, ax = plt.subplots()
            ax.plot(costs)
            plt.text(0.6, 0.9, "mean:{:.2f}, std:{:.2f}".format(
                mean, std), transform=ax.transAxes)
            plt.ylabel("time(ms)")
            plt.show()

    def task_cost_mean(self, task_name):
        """Time cost mean of task_name
        """
        task_costs = self.task_cost_dict.get(task_name, None)
        costs = []
        if task_costs:
            costs = [task_cost.total() for task_cost in task_costs]
        return statistics.fmean(costs)

    def task_cost_std(self, task_name):
        """Time cost std of task_name
        """
        task_costs = self.task_cost_dict.get(task_name, None)
        costs = []
        if task_costs:
            costs = [task_cost.total() for task_cost in task_costs]
        return statistics.stdev(costs)

    def max(self):
        """multi-node tree to find leaf node, find max time cost node
        """
        pass

    def min(self):
        """multi-node tree to find leaf node, find min time cost node
        """
        pass

    def max_k(self):
        """multi-node tree to find leaf node, find max k time cost node
        """
        pass

    def min_k(self):
        """multi-node tree to find leaf node, find min k time cost node
        """
        pass

    def large_than(self, threshold):
        """multi-node tree to find leaf node, find time cost node large than
           threshold
        """
        pass
