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
#include "modules/planning/proto/learning_data.pb.h"

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
  void WriteOutLearningData(const LearningData& learning_data,
                            const std::string& file_name);
  void GenerateTrajectoryLabel(
      const std::list<apollo::localization::LocalizationEstimate>&
          localization_for_label,
      LearningDataFrame* learning_data_frame);

  LearningDataFrame* learning_data_frame_ = nullptr;  // not owned
  LearningData learning_data_;
  int learning_data_file_index_ = 0;
  std::list<apollo::localization::LocalizationEstimate>
      localization_for_label_;
  int total_learning_data_frame_num_ = 0;
};

}  // namespace planning
}  // namespace apollo
