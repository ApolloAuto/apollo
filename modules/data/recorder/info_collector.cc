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

#include "modules/data/recorder/info_collector.h"

#include "modules/canbus/common/canbus_gflags.h"
#include "modules/common/adapters/adapter_manager.h"

DEFINE_string(task_info_template_file,
              "modules/data/conf/task_info_template.pb.txt",
              "Path of the task info template file.");

namespace apollo {
namespace data {

using apollo::common::util::GetProtoFromASCIIFile;
using apollo::common::util::GetProtoFromFile;
using apollo::common::util::SetProtoToASCIIFile;

InfoCollector::InfoCollector() : task_info_(LoadTaskInfoTemplate()) {
}

const Task &InfoCollector::GetTaskInfo() {
  // Use MergeFrom to override the template.
  GetVehicleInfo();
  GetEnvironmentInfo();
  GetHardwareInfo();
  GetSoftwareInfo();
  GetUserInfo();
  return task_info_;
}

const VehicleInfo &InfoCollector::GetVehicleInfo() {
  VehicleInfo *vehicle = task_info_.mutable_vehicle();
  static auto *chassis_detail = CHECK_NOTNULL(
      apollo::common::adapter::AdapterManager::GetChassisDetail());
  if (!chassis_detail->Empty()) {
    *vehicle->mutable_license() = chassis_detail->GetLatestObserved().license();
  }

  CHECK(GetProtoFromFile(FLAGS_canbus_conf_file,
                         vehicle->mutable_canbus_conf()));
  CHECK(GetProtoFromFile(FLAGS_vehicle_config_path,
                         vehicle->mutable_vehicle_config()));
  return *vehicle;
}

// TODO(xiaoxq): Implement the info getters.
const EnvironmentInfo &InfoCollector::GetEnvironmentInfo() {
  return task_info_.environment();
}

const HardwareInfo &InfoCollector::GetHardwareInfo() {
  return task_info_.hardware();
}

const SoftwareInfo &InfoCollector::GetSoftwareInfo() {
  SoftwareInfo *software = task_info_.mutable_software();
  if (const char* docker_image = std::getenv("DOCKER_IMG")) {
    software->set_docker_image(docker_image);
  }
  return *software;
}

const UserInfo &InfoCollector::GetUserInfo() {
  return task_info_.user();
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
