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

/**
 * @file dp_poly_path_optimizer.h
 **/

#ifndef MODULES_PLANNING_TASKS_PATH_DECIDER_PATH_DECIDER_H_
#define MODULES_PLANNING_TASKS_PATH_DECIDER_PATH_DECIDER_H_

#include <string>

#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {

class PathDecider : public Task {
 public:
  explicit PathDecider(const std::string &name);

  apollo::common::Status Execute(
      Frame *frame, ReferenceLineInfo *reference_line_info) override;

 private:
  apollo::common::Status Process(const PathData &path_data,
                                 PathDecision *const path_decision);

  bool ComputeBoundingBoxesForAdc(const FrenetFramePath &frenet_frame_path,
                                  const std::size_t evaluate_time_slots,
                                  std::vector<common::math::Box2d> *adc_boxes);

  bool MakeObjectDecision(const PathData &path_data,
                          PathDecision *const path_decision);

  bool MakeStaticObstacleDecision(const PathData &path_data,
                                  PathDecision *const path_decision);

  bool MakeDynamicObstcleDecision(const PathData &path_data,
                                  PathDecision *const path_decision);

  const ReferenceLine *reference_line_ = nullptr;
  const SpeedData *speed_data_ = nullptr;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_PATH_DECIDER_PATH_DECIDER_H_
