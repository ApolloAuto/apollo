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

#include "gflags/gflags.h"
#include "modules/common/log.h"
#include "ros/include/ros/ros.h"

#include "modules/perception/tool/export_sensor_data/export_sensor_data.h"

DECLARE_string(flagfile);

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "export_sensor_data");
  ros::AsyncSpinner spinner(4);
  AINFO << "Start export_sensor_data.";
  FLAGS_flagfile =
      "./modules/perception/tool/export_sensor_data/conf/"
      "export_sensor_data.flag";
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  apollo::perception::ExportSensorData export_sensor_data;
  export_sensor_data.Init();
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
