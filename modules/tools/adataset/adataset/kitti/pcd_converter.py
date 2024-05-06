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
'''KITTI pcd file to pcl pcd file converter.'''

import logging
import numpy as np

from record_msg import pypcd

def convert_pcd(input_file, output_file):
  # Loads LIDAR data from binary numpy format.
  # Data is stored as (x, y, z, intensity).
  scan = np.fromfile(input_file, dtype=np.dtype([
    ('x', np.float32),
    ('y', np.float32),
    ('z', np.float32),
    ('intensity', np.float32)]))

  logging.debug("points: {},{}".format(np.shape(scan), scan.dtype))
  point_cloud = pypcd.PointCloud.from_array(scan)
  point_cloud.save(output_file)
  print("Success! Pcd file saved to '{}'".format(output_file))
