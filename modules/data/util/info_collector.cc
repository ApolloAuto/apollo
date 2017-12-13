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

#include "modules/canbus/common/canbus_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/control/common/control_gflags.h"

DEFINE_string(task_info_template_file,
              "modules/data/conf/task_info_template.pb.txt",
              "Path of the task info template file.");

DEFINE_string(recorder_conf_file, "modules/data/conf/recorder_conf.pb.txt",
              "Path of the recorder config file.");

namespace apollo {
namespace data {
namespace {

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

InfoCollector::InfoCollector() : task_info_(LoadTaskInfoTemplate()) {
  CHECK(GetProtoFromASCIIFile(FLAGS_recorder_conf_file, &config_));
}

const Task &InfoCollector::GetTaskInfo() {
  // Use MergeFrom to override the template.
  GetVehicleInfo();
  GetEnvironmentInfo();
  GetHardwareInfo();
  GetSoftwareInfo();
  GetUserInfo();
  return instance()->task_info_;
}

const VehicleInfo &InfoCollector::GetVehicleInfo() {
  VehicleInfo *vehicle = instance()->task_info_.mutable_vehicle();
  static auto *chassis_detail = CHECK_NOTNULL(
      apollo::common::adapter::AdapterManager::GetChassisDetail());

  chassis_detail->Observe();
  if (!chassis_detail->Empty()) {
    *vehicle->mutable_license() = chassis_detail->GetLatestObserved().license();
  }

  CHECK(GetProtoFromASCIIFile(FLAGS_canbus_conf_file,
                              vehicle->mutable_canbus_conf()));
  CHECK(GetProtoFromASCIIFile(FLAGS_vehicle_config_path,
                              vehicle->mutable_vehicle_config()));
  CHECK(GetProtoFromASCIIFile(FLAGS_control_conf_file,
                              vehicle->mutable_control_config()));
  return *vehicle;
}

// TODO(xiaoxq): Implement the info getters.
const EnvironmentInfo &InfoCollector::GetEnvironmentInfo() {
  return instance()->task_info_.environment();
}

const HardwareInfo &InfoCollector::GetHardwareInfo() {
  HardwareInfo *hardware = instance()->task_info_.mutable_hardware();
  *hardware->mutable_configs() =
      LoadFiles(instance()->config_.hardware_configs());
  return *hardware;
}

const SoftwareInfo &InfoCollector::GetSoftwareInfo() {
  SoftwareInfo *software = instance()->task_info_.mutable_software();
  if (const char* docker_image = std::getenv("DOCKER_IMG")) {
    software->set_docker_image(docker_image);
  }
  *software->mutable_configs() =
      LoadFiles(instance()->config_.software_configs());
  return *software;
}

const UserInfo &InfoCollector::GetUserInfo() {
  return instance()->task_info_.user();
}

Task InfoCollector::LoadTaskInfoTemplate() {
  Task task_info;
  // The template might not exist, then just ignore.
  if (apollo::common::util::PathExists(FLAGS_task_info_template_file)) {
    CHECK(GetProtoFromASCIIFile(FLAGS_task_info_template_file, &task_info));
  }
  return task_info;
}

bool InfoCollector::SaveTaskInfoTemplate(const Task &task_info) {
  return SetProtoToASCIIFile(task_info, FLAGS_task_info_template_file);
}

}  // namespace data
}  // namespace apollo
