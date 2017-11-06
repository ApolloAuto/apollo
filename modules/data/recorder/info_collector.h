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

#ifndef MODULES_DATA_RECORDER_INFO_COLLECTOR_H_
#define MODULES_DATA_RECORDER_INFO_COLLECTOR_H_

#include "modules/data/proto/task.pb.h"

/**
 * @namespace apollo::data
 * @brief apollo::data
 */
namespace apollo {
namespace data {

class InfoCollector {
 public:
  InfoCollector();

  // Get task information.
  const Task &GetTaskInfo();

  // Get specific information.
  // Listening topics: ChassisDetail.
  const VehicleInfo &GetVehicleInfo();
  const EnvironmentInfo &GetEnvironmentInfo();
  const HardwareInfo &GetHardwareInfo();
  const SoftwareInfo &GetSoftwareInfo();
  const UserInfo &GetUserInfo();

  // Load and save the task information template.
  static Task LoadTaskInfoTemplate();
  static bool SaveTaskInfoTemplate(const Task &task_info);

 private:
  Task task_info_;
};

}  // namespace data
}  // namespace apollo

#endif  // MODULES_DATA_RECORDER_INFO_COLLECTOR_H_
