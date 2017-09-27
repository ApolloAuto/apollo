#!/usr/bin/env python

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
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

import matplotlib.pyplot as plt

from map import Map
from localization import Localization

if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="Mapshow is a tool to display hdmap info on a map.",
        prog="mapshow.py")

    parser.add_argument(
        "-m", "--map", action="store", type=str, required=True,
        help="Specify the map file in txt or binary format")
    parser.add_argument(
        "-sl", "--showlaneids", action="store_const", const=True,
        help="Show all lane ids in map")
    parser.add_argument(
        "-ss", "--showsignals", action="store_const", const=True,
        help="Show all signal light stop lines with ids in map")
    parser.add_argument(
        "-l", "--laneid", nargs='+',
        help="Show specific lane id(s) in map")
    parser.add_argument(
        "--loc", action="store", type=str, required=False,
        help="Specify the localization pb file in txt format")

    args = parser.parse_args()

    map = Map()
    map.load(args.map)
    lane_ids = args.laneid
    if lane_ids is None:
        lane_ids = []
    map.draw_lanes(plt, args.showlaneids, lane_ids)
    if args.showsignals:
        map.draw_signal_lights(plt)

    if args.loc is not None:
        localization = Localization()
        localization.load(args.loc)
        localization.plot_vehicle(plt)

    plt.axis('equal')
    plt.show()
