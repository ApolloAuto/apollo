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

namespace apollo {
namespace data {

Task InfoCollector::GetTaskInfo() {
  Task task;
  *task.mutable_vehicle() = GetVehicleInfo();
  *task.mutable_environment() = GetEnvironmentInfo();
  *task.mutable_hardware() = GetHardwareInfo();
  *task.mutable_software() = GetSoftwareInfo();
  *task.mutable_user() = GetUserInfo();
  return task;
}

// TODO(xiaoxq): Implement the info getters.
VehicleInfo InfoCollector::GetVehicleInfo() {
  VehicleInfo vehicle;
  return vehicle;
}

EnvironmentInfo InfoCollector::GetEnvironmentInfo() {
  EnvironmentInfo environment;
  return environment;
}

HardwareInfo InfoCollector::GetHardwareInfo() {
  HardwareInfo hardware;
  return hardware;
}

SoftwareInfo InfoCollector::GetSoftwareInfo() {
  SoftwareInfo software;
  return software;
}

UserInfo InfoCollector::GetUserInfo() {
  UserInfo user;
  return user;
}

}  // namespace data
}  // namespace apollo
