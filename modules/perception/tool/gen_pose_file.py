#!/usr/bin/python

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

# Default PCD file directory
PCD_DIR_PATH = '/apollo/data/pcd'

def process_pose_file(pcd_dir):
    # Pose file directory
    pose_dir = os.path.join(os.path.dirname(pcd_dir), 'pose')
    if not os.path.exists(pose_dir):
        os.mkdir(pose_dir)

    pose_info_file = os.path.join(pcd_dir, 'pose.txt')
    assert os.path.exists(pose_info_file), 'The file of pose info does not exist'
    with open(pose_info_file, 'r') as f_in:
        for line in f_in:
            out_pose_file = os.path.join(pose_dir, line.split()[0] + '.pose')
            with open(out_pose_file, 'w') as f_out:
                f_out.write(line)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: %s <PCD_DIRECTORY>" % sys.argv[0])
        sys.exit(1)
    else if len(sys.argv) == 2:
        pcd_dir = sys.argv[1]
    else:
        pcd_dir = PCD_DIR_PATH

    process_pose_file(pcd_dir)
