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

import sys
import os

def main(argv):
    if len(argv) < 3:
        print("The number of input parameter should be empty or only one.")

    # Use the default PCD file directory if that is not specified.
    pcd_dir = argv[1] if len(argv) == 2 or "/apollo/data/pcd"
    # Pose file directory
    pose_dir = os.path.join(os.path.dirname(pcd_dir), "pose")
    if not os.path.exists(pose_dir):
        os.mkdir(pose_dir)

    pose_info_file = os.path.join(pcd_dir, "pose.txt")
    with open(pose_info_file, 'r') as f_in:
        for line in f_in:
            out_pose_file = os.path.join(pose_dir, line.split()[0] + ".pose")
            with open(out_pose_file, 'w') as f_out:
                f_out.write(line)

if __name__ == "__main__":
    main(sys.argv)
