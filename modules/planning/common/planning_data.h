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
 * @file
 **/

#ifndef MODULES_PLANNING_COMMON_PLANNING_DATA_H_
#define MODULES_PLANNING_COMMON_PLANNING_DATA_H_

#include <memory>
#include <string>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/planning/common/decision_data.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/planning_data.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"

namespace apollo {
namespace planning {

class PlanningData {
 public:
  PlanningData() = default;

  void set_decision_data(std::shared_ptr<DecisionData>& decision_data);

  const DecisionData& decision_data() const;

  DecisionData* mutable_decision_data() const;

  const PathData& path_data() const;
  const SpeedData& speed_data() const;

  PathData* mutable_path_data();
  SpeedData* mutable_speed_data();

  double timestamp() const;

  // aggregate final result together by some configuration
  bool aggregate(const double time_resolution, const double relative_time,
                 PublishableTrajectory* PublishableTrajectory);

  std::string DebugString() const;

 protected:
  std::shared_ptr<DecisionData> decision_data_ = nullptr;
  PathData path_data_;
  SpeedData speed_data_;
  double timestamp_ = 0.0;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_PLANNING_DATA_H_
