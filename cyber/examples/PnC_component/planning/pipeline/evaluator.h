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

#include <chrono>
#include <string>

#include "modules/planning/common/trajectory_evaluator.h"
#include "modules/planning/proto/learning_data.pb.h"

namespace apollo {
namespace planning {

class Evaluator {
 public:
  void Init();
  void Close();

  void Evaluate(const std::string& source_file);

 private:
  void WriteOutData(const std::string& source_filename,
                    const LearningData& learning_data);

 private:
  std::chrono::time_point<std::chrono::system_clock> start_time_;
  LearningData learning_data_;
  TrajectoryEvaluator trajectory_evaluator_;
};

}  // namespace planning
}  // namespace apollo
