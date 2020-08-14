#!/usr/bin/env python3

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
from modules.tools.mapshow.libs.map import Map

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Raodshow is a tool to display road info on a map.",
        prog="roadshow.py")

    parser.add_argument(
        "-m", "--map", action="store", type=str, required=True,
        help="Specify the map file in txt or binary format")

    args = parser.parse_args()

    map = Map()
    map.load(args.map)
    map.draw_roads(plt)

    plt.axis('equal')
    plt.show()
