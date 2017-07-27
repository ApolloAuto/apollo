/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "velodyne_pointcloud/pcd_exporter.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcd_exporter");

  ros::NodeHandle hn("~");
  ros::NodeHandle n;

  apollo::drivers::velodyne::PCDExporter exporter(n, hn);
  exporter.init();
  ros::spin();

  return 0;
}
