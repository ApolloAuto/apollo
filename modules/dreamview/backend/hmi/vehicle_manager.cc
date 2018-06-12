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

#include "gflags/gflags.h"
#include "modules/common/util/file.h"
#include "modules/common/util/string_util.h"

DEFINE_string(vehicle_data_config_filename,
              "/apollo/modules/dreamview/conf/vehicle_data.pb.txt",
              "Vehicle data config file.");

namespace apollo {
namespace dreamview {

using apollo::common::util::GetProtoFromFile;
using apollo::common::util::StrCat;

VehicleManager::VehicleManager() {
  CHECK(GetProtoFromFile(FLAGS_vehicle_data_config_filename, &vehicle_data_))
      << "Unable to parse VehicleData config file "
      << FLAGS_vehicle_data_config_filename;
  for (auto &data_file : *vehicle_data_.mutable_data_files()) {
    data_file.set_dest_path(
        apollo::common::util::TranslatePath(data_file.dest_path()));
  }
}

bool VehicleManager::UseVehicle(const std::string &vehicle_data_path) {
  if (!apollo::common::util::DirectoryExists(vehicle_data_path)) {
    AERROR << "Cannot find vehicle data: " << vehicle_data_path;
    return false;
  }

  for (const auto &data_file : vehicle_data_.data_files()) {
    const auto source_path =
        StrCat(vehicle_data_path, "/", data_file.source_path());
    const auto &dest_path = data_file.dest_path();

    const bool ret = apollo::common::util::Copy(source_path, dest_path);
    AINFO_IF(ret) << "Copied " << source_path << " to " << dest_path;
  }

  static const std::string kBroadcastExtrinsicsCmd =
      "bash scripts/broadcast_extrinsics.sh";
  const int ret = std::system(kBroadcastExtrinsicsCmd.c_str());
  AERROR_IF(ret != 0) << "Command returns " << ret << ": "
                      << kBroadcastExtrinsicsCmd;

  return true;
}

}  // namespace dreamview
}  // namespace apollo
