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
 * @file frame.h
 **/

#ifndef MODULES_PLANNING_COMMON_FRAME_H_
#define MODULES_PLANNING_COMMON_FRAME_H_

#include <cstdint>
#include <memory>
#include <string>

#include "modules/planning/common/planning_data.h"

namespace apollo {
namespace planning {

class Frame {
 public:
  explicit Frame(uint32_t sequence_num);

  void set_sequence_num(const uint32_t sequence_num);

  uint32_t sequence_num() const;
  const PlanningData &planning_data() const;

  PlanningData *mutable_planning_data();
  std::string DebugString() const;

  void set_computed_trajectory(const PublishableTrajectory &trajectory);
  const PublishableTrajectory &computed_trajectory() const;

 private:
  uint32_t _sequence_num;
  PublishableTrajectory _computed_trajectory;
  PlanningData _planning_data;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_FRAME_H_
