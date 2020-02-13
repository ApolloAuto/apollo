/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <list>
#include <string>

#include "cyber/common/file.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/planning/proto/instance.pb.h"

namespace apollo {
namespace planning {

class FeatureGenerator {
 public:
  void Init();
  void Close();

  void ProcessOfflineData(const std::string& record_filename);

 private:
  void OnLocalization(const apollo::localization::LocalizationEstimate& le);
  void OnChassis(const apollo::canbus::Chassis& chassis);
  void WriteOutInstances(const Instances& instances,
                         const std::string& file_name);
  void GenerateTrajectoryLabel(
      const std::list<apollo::localization::LocalizationEstimate>&
          localization_for_label,
      Instance* instance);

  Instance* instance_ = nullptr;  // not owned
  Instances instances_;
  int instance_file_index_ = 0;
  std::list<apollo::localization::LocalizationEstimate>
      localization_for_label_;
  int total_instance_num_ = 0;
};

}  // namespace planning
}  // namespace apollo
