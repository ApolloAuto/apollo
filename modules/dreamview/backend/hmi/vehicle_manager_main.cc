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

#include "modules/dreamview/backend/hmi/vehicle_manager.h"

#include "cyber/common/log.h"
#include "gflags/gflags.h"

DEFINE_string(vehicle_data_path, "modules/calibration/data/mkz_example",
              "Vehicle data path.");

int main(int argc, char **argv) {
  FLAGS_logtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  apollo::dreamview::VehicleManager::Instance()->UseVehicle(
      FLAGS_vehicle_data_path);
  AINFO << "Switched to vehicle with data from " << FLAGS_vehicle_data_path;

  return 0;
}
