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

from datetime import datetime
import json
import sys

from modules.map.relative_map.proto import navigation_pb2


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: python generator.py "
              "navi_line1.smoothed navi_line2.smoothed ...")
        sys.exit(0)
    navi_files = sys.argv[1:]
    # generate navigation info
    navigation_info = navigation_pb2.NavigationInfo()
    priority = 0
    for fdata in navi_files:
        print("processing " + fdata)
        navigation_path = navigation_info.navigation_path.add()
        navigation_path.path_priority = priority
        priority += 1
        navigation_path.path.name = "navigation"

        with open(fdata, 'r') as f:
            cnt = 0
            for line in f:
                cnt += 1
                if cnt < 3:
                    continue
                json_point = json.loads(line)
                point = navigation_path.path.path_point.add()
                point.x = json_point['x']
                point.y = json_point['y']
                point.s = json_point['s']
                point.theta = json_point['theta']
                point.kappa = json_point['kappa']
                point.dkappa = json_point['dkappa']

    datetime_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    with open(datetime_str + ".dbmap", 'w') as f:
        f.write(str(navigation_info))
