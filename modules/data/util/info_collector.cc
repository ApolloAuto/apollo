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

#include "modules/data/util/info_collector.h"

#include <string>

#include "gflags/gflags.h"
#include "modules/common/kv_db/kv_db.h"
#include "modules/common/util/file.h"

DEFINE_string(static_info_conf_file,
              "modules/data/conf/static_info_conf.pb.txt",
              "Path of the StaticInfo config file.");

namespace apollo {
namespace data {
namespace {

using apollo::common::KVDB;
using apollo::common::util::GetProtoFromASCIIFile;
using apollo::common::util::SetProtoToASCIIFile;
using apollo::common::util::TranslatePath;
using google::protobuf::Map;
using google::protobuf::RepeatedPtrField;

// Load files list to {file_path: file_content} map.
Map<std::string, std::string> LoadFiles(
    const RepeatedPtrField<std::string> &files) {
  Map<std::string, std::string> result;
  std::string content;
  for (const auto &file : files) {
    if (apollo::common::util::GetContent(TranslatePath(file), &content)) {
      result.insert({file, content});
    } else {
      AERROR << "Cannot load file " << file;
    }
  }
  return result;
}

}  // namespace

InfoCollector::InfoCollector() {
  CHECK(GetProtoFromASCIIFile(FLAGS_static_info_conf_file, &config_));
}

const StaticInfo &InfoCollector::GetStaticInfo() {
  // Use MergeFrom to override the template.
  GetVehicleInfo();
  GetEnvironmentInfo();
  GetHardwareInfo();
  GetSoftwareInfo();
  GetUserInfo();
  return instance()->static_info_;
}

const VehicleInfo &InfoCollector::GetVehicleInfo() {
  VehicleInfo *vehicle = instance()->static_info_.mutable_vehicle();

  const std::string vehicle_name = KVDB::Get("apollo:dreamview:vehicle");
  if (!vehicle_name.empty()) {
    vehicle->set_name(vehicle_name);
  }

  const std::string vehicle_vin = KVDB::Get("apollo:canbus:vin");
  if (!vehicle_vin.empty()) {
    vehicle->mutable_license()->set_vin(vehicle_vin);
  }

  return *vehicle;
}

const EnvironmentInfo &InfoCollector::GetEnvironmentInfo() {
  EnvironmentInfo *environment = instance()->static_info_.mutable_environment();

  const std::string map_name = KVDB::Get("apollo:dreamview:map");
  if (!map_name.empty()) {
    environment->set_map_name(map_name);
  }
  return *environment;
}

const HardwareInfo &InfoCollector::GetHardwareInfo() {
  HardwareInfo *hardware = instance()->static_info_.mutable_hardware();
  *hardware->mutable_configs() =
      LoadFiles(instance()->config_.hardware_configs());
  return *hardware;
}

const SoftwareInfo &InfoCollector::GetSoftwareInfo() {
  SoftwareInfo *software = instance()->static_info_.mutable_software();
  if (const char* docker_image = std::getenv("DOCKER_IMG")) {
    software->set_docker_image(docker_image);
  }

  const std::string commit_id = KVDB::Get("apollo:data:commit_id");
  if (!commit_id.empty()) {
    software->set_commit_id(commit_id);
  }

  const std::string mode_name = KVDB::Get("apollo:dreamview:mode");
  if (!mode_name.empty()) {
    software->set_mode(mode_name);
  }

  *software->mutable_configs() =
      LoadFiles(instance()->config_.software_configs());
  return *software;
}

const UserInfo &InfoCollector::GetUserInfo() {
  return instance()->static_info_.user();
}

}  // namespace data
}  // namespace apollo
