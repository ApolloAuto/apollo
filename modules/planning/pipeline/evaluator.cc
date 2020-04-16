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

#include "modules/planning/pipeline/evaluator.h"

#include <string>

#include "cyber/common/file.h"
// #include "modules/common/adapters/adapter_gflags.h"
// #include "modules/common/util/point_factory.h"
// #include "modules/common/util/util.h"
// #include "modules/map/hdmap/hdmap_util.h"
// #include "modules/map/proto/map_lane.pb.h"
// #include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
// #include "modules/planning/common/util/math_util.h"

DEFINE_string(planning_data_dir, "/apollo/modules/planning/data/",
              "Prefix of files to store learning_data_frame data");
DEFINE_int32(planning_freq, 10, "frequence of planning message");

namespace apollo {
namespace planning {

void Evaluator::Init() {
}

void Evaluator::Evaluate(const std::string& source_file) {
  source_filename_ =
      source_file.substr(source_file.find_last_of("/") + 1);
  cyber::common::GetProtoFromFile(source_file,
                                  &learning_data_);

  EvaluateADCTrajectory();
}

void Evaluator::WriteOutLearningData(
    const LearningData& learning_data) {
  const std::string file_name =
      FLAGS_planning_data_dir + source_filename_;
  cyber::common::SetProtoToBinaryFile(learning_data, file_name);
  cyber::common::SetProtoToASCIIFile(learning_data, file_name + ".txt");
  learning_data_.Clear();
}

void Evaluator::Close() {
  WriteOutLearningData(learning_data_);
}

void Evaluator::EvaluateTrajectoryByTime(
    const std::vector<common::TrajectoryPoint> &trajectory,
    const double relative_time) {
  // TODO(all)
}

void Evaluator::EvaluateADCTrajectory() {
  // TODO(all)
}


}  // namespace planning
}  // namespace apollo
