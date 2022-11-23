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
import os
import sys
import logging

from dataset_converter import convert_dataset
from pcd_converter import convert_pcd


def main(args=sys.argv):
  parser = argparse.ArgumentParser(
    description="nuScenes dataset convert to record tool.",
    prog="main.py")

  parser.add_argument(
    "-i", "--input", action="store", type=str, required=True,
    nargs="?", const="", help="Input file or directory.")
  parser.add_argument(
    "-o", "--output", action="store", type=str, required=False,
    nargs="?", const="", help="Output file or directory.")

  args = parser.parse_args(args[1:])
  logging.debug(args)

  if os.path.isdir(args.input):
    if args.output is None:
      args.output = '.'
    convert_dataset(args.input, args.output)
  elif os.path.isfile(args.input):
    if args.output is None:
      args.output = 'result.pcd'
    convert_pcd(args.input, args.output)
  else:
    logging.error("Input error! '{}'".format(args.input))


if __name__ == '__main__':
  main()
