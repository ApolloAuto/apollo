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
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/kv_db/kv_db.h"
#include "modules/common/util/file.h"
#include "modules/routing/proto/routing.pb.h"

DEFINE_string(static_info_conf_file,
              "/apollo/modules/data/conf/static_info_conf.pb.txt",
              "Path of the StaticInfo config file.");

namespace apollo {
namespace data {
namespace {

using apollo::common::KVDB;
using apollo::common::util::GetProtoFromASCIIFile;
using google::protobuf::Map;
using google::protobuf::RepeatedPtrField;

// Load files list to {file_path: file_content} map.
Map<std::string, std::string> LoadFiles(
    const RepeatedPtrField<std::string> &files) {
  Map<std::string, std::string> result;
  std::string content;
  for (const auto &file : files) {
    if (apollo::common::util::GetContent(file, &content)) {
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

void InfoCollector::Init(const std::shared_ptr<apollo::cyber::Node>& node) {
  Instance()->routing_request_reader_ =
      node->CreateReader<apollo::routing::RoutingRequest>(
          FLAGS_routing_request_topic);
}

const StaticInfo &InfoCollector::GetStaticInfo() {
  // Use MergeFrom to override the template.
  GetVehicleInfo();
  GetEnvironmentInfo();
  GetHardwareInfo();
  GetSoftwareInfo();
  GetUserInfo();
  return Instance()->static_info_;
}

const VehicleInfo &InfoCollector::GetVehicleInfo() {
  VehicleInfo *vehicle = Instance()->static_info_.mutable_vehicle();

  const std::string vehicle_vin = KVDB::Get("apollo:canbus:vin");
  if (!vehicle_vin.empty()) {
    vehicle->mutable_license()->set_vin(vehicle_vin);
  }

  return *vehicle;
}

const EnvironmentInfo &InfoCollector::GetEnvironmentInfo() {
  // TODO(xiaoxq): Populate information like temperature, etc.
  return Instance()->static_info_.environment();
}

const HardwareInfo &InfoCollector::GetHardwareInfo() {
  HardwareInfo *hardware = Instance()->static_info_.mutable_hardware();
  *hardware->mutable_configs() =
      LoadFiles(Instance()->config_.hardware_configs());
  return *hardware;
}

const SoftwareInfo &InfoCollector::GetSoftwareInfo() {
  SoftwareInfo *software = Instance()->static_info_.mutable_software();

  const std::string commit_id = KVDB::Get("apollo:data:commit_id");
  if (!commit_id.empty()) {
    software->set_commit_id(commit_id);
  }

  *software->mutable_configs() =
      LoadFiles(Instance()->config_.software_configs());

  const auto routing_request_reader = Instance()->routing_request_reader_;
  if (routing_request_reader != nullptr) {
    routing_request_reader->Observe();
    const auto routing_request = routing_request_reader->GetLatestObserved();
    if (routing_request != nullptr) {
      *software->mutable_latest_routing_request() = *routing_request;
    } else {
      AERROR << "No RoutingRequest has been received.";
    }
  } else {
    AERROR << "RoutingRequest observer is not inited.";
  }
  return *software;
}

const UserInfo &InfoCollector::GetUserInfo() {
  // TODO(xiaoxq): Populate information like driver, co-dirver, etc.
  return Instance()->static_info_.user();
}

}  // namespace data
}  // namespace apollo
