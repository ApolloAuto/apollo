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

import argparse
import logging
import sys

from aperf.profiler import Profiler


def main(args=sys.argv):
    """main funciton

    Args:
        args (_type_, optional): _description_. Defaults to sys.argv.
    """
    parser = argparse.ArgumentParser(
        description="Apollo performance statistics tool.",
        prog="main.py")

    parser.add_argument(
        "command", action="store", choices=['cost', 'timeline'],
        type=str, nargs="?", const="", default="list",
        help="aperf command list.")
    parser.add_argument(
        "-f", "--file", action="store", type=str, required=True, nargs='?',
        const="", help="perf file.")
    parser.add_argument(
        "-t", "--task_name", action="store", type=str, required=False,
        nargs="?", const="", help="task name.")

    args = parser.parse_args(args[1:])
    logging.debug(args)

    profiler = Profiler()
    profiler.parse(args.file)
    if args.command == "cost":
        profiler.draw_timecost(args.task_name)
    elif args.command == "timeline":
        profiler.draw_timeline()
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
