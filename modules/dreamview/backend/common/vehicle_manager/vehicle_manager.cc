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

#include "modules/dreamview/backend/common/vehicle_manager/vehicle_manager.h"

#include <cstdlib>

#include "absl/strings/str_cat.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"

namespace apollo {
namespace dreamview {

using cyber::common::GetProtoFromFile;

VehicleManager::VehicleManager() {
  ACHECK(GetProtoFromFile(FLAGS_vehicle_data_config_filename, &vehicle_data_))
      << "Unable to parse VehicleData config file "
      << FLAGS_vehicle_data_config_filename;
}

const std::string &VehicleManager::GetVehicleDataPath() const {
  return vehicle_data_path_;
}

bool VehicleManager::UseVehicle(const std::string &vehicle_data_path) {
  if (FLAGS_vehicle_changed_use_copy_mode) {
    return UseVehicleUseCopyMode(vehicle_data_path);
  } else {
    return UseVehicleUseSymlinkMode(vehicle_data_path);
  }
}

bool VehicleManager::UseVehicleUseCopyMode(
    const std::string &vehicle_data_path) {
  if (!cyber::common::DirectoryExists(vehicle_data_path)) {
    AERROR << "Cannot find vehicle data: " << vehicle_data_path;
    return false;
  }
  vehicle_data_path_ = vehicle_data_path;

  for (const auto &data_file : vehicle_data_.data_files()) {
    const auto source_path =
        absl::StrCat(vehicle_data_path, "/", data_file.source_path());
    const auto &dest_path = data_file.dest_path();

    const bool ret = cyber::common::Copy(source_path, dest_path);
    AINFO_IF(ret) << "Copied " << source_path << " to " << dest_path;
  }
  AINFO << "Use vehice [" << vehicle_data_path << "] copy config successfully.";

  // Reload vehicle config for current process.
  apollo::common::VehicleConfigHelper::Init();

  return true;
}

bool VehicleManager::UseVehicleUseSymlinkMode(
    const std::string &vehicle_data_path) {
  std::string vehicle_type = vehicle_data_path;
  auto pos = vehicle_data_path.find_last_of('/');
  if (pos != std::string::npos) {
    vehicle_type = vehicle_data_path.substr(pos + 1);
  }
  std::string cmd = "aem profile use " + vehicle_type;
  AINFO << "Use vehicle by cmd: " << cmd;
  const int ret = std::system(cmd.data());
  if (ret == 0) {
    AINFO << "SUCCESS: use vehicle " << vehicle_type;

    // Reload vehicle config for current process.
    apollo::common::VehicleConfigHelper::Init();
    return true;
  } else {
    AERROR << "FAILED: use vehicle " << vehicle_type;
    return false;
  }
}

}  // namespace dreamview
}  // namespace apollo
