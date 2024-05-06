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
"""
amodel main function
"""

import argparse
import logging
import sys

from amodel.model_manage import (
  amodel_install,
  amodel_remove,
  amodel_list,
  amodel_info)


def main(args=sys.argv):
  parser = argparse.ArgumentParser(
    description="Apollo perception model management tool.",
    prog="main.py")

  parser.add_argument(
    "command", action="store", choices=['list', 'info', 'install', 'remove'],
    type=str, nargs="?", const="", default="list", help="amodel command list.")
  parser.add_argument(
    "model_name", action="store", type=str, nargs="?", const="",
    help="model name or install path.")
  parser.add_argument(
    "-s", "--skip", action="store", type=bool, required=False, nargs='?',
    const=True, help="Skip install when model exist.")

  args = parser.parse_args(args[1:])
  logging.debug(args)

  if args.command == "install":
    amodel_install(args.model_name, args.skip)
  elif args.command == "remove":
    amodel_remove(args.model_name)
  elif args.command == "list":
    amodel_list()
  elif args.command == "info":
    amodel_info(args.model_name)
  else:
    parser.print_help()

if __name__ == "__main__":
  main()
