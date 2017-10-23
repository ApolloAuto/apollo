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

#include "modules/dreamview/backend/hmi/hmi.h"

#include "google/protobuf/util/json_util.h"
#include "third_party/json/json.hpp"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/util/util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"

namespace apollo {
namespace dreamview {

using apollo::common::adapter::AdapterManager;
using Json = nlohmann::json;

HMI::HMI(WebSocketHandler *websocket) : websocket_(websocket) {
  CHECK(common::util::GetProtoFromFile(FLAGS_hmi_config_filename, &config_))
      << "Unable to parse HMI config file " << FLAGS_hmi_config_filename;

  // Init status of modules and hardware.
  for (const auto &module : config_.modules()) {
    status_.add_modules()->set_name(module.name());
  }
  for (const auto &hardware : config_.hardware()) {
    status_.add_hardware()->set_name(hardware.name());
  }
}

void HMI::Start() {
  AdapterManager::AddHMIStatusCallback(&HMI::OnHMIStatus, this);
}

void HMI::OnHMIStatus(const HMIStatus &hmi_status) {
  // Note that we only merged status of modules and hardware, which matches the
  // hmi_status_helper.
  for (const auto &new_status : hmi_status.modules()) {
    auto *old_status = GetModuleStatus(new_status.name());
    if (old_status) {
      old_status->MergeFrom(new_status);
    } else {
      AERROR << "Updating HMIStatus, unknown module " << new_status.name();
    }
  }
  for (const auto &new_status : hmi_status.hardware()) {
    auto *old_status = GetHardwareStatus(new_status.name());
    if (old_status) {
      old_status->MergeFrom(new_status);
    } else {
      AERROR << "Updating HMIStatus, unknown hardware " << new_status.name();
    }
  }

  BroadcastHMIStatus();
}

void HMI::BroadcastHMIStatus() const {
  // In unit tests, we may leave websocket_ as NULL and skip broadcasting.
  if (websocket_ == nullptr) {
    return;
  }

  // Broadcast jsonified HMIStatus to clients via websocket.
  std::string status_json_string;
  google::protobuf::util::MessageToJsonString(status_, &status_json_string);

  Json status_json;
  status_json["type"] = "HMIStatus";
  status_json["data"] = Json::parse(status_json_string);
  websocket_->BroadcastData(status_json.dump());
}

ModuleStatus* HMI::GetModuleStatus(const std::string &module_name) {
  for (auto &module_status : *status_.mutable_modules()) {
    if (module_status.name() == module_name) {
      return &module_status;
    }
  }
  return nullptr;
}

HardwareStatus* HMI::GetHardwareStatus(const std::string &hardware_name) {
  for (auto &hardware_status : *status_.mutable_hardware()) {
    if (hardware_status.name() == hardware_name) {
      return &hardware_status;
    }
  }
  return nullptr;
}

}  // namespace dreamview
}  // namespace apollo
