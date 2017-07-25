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

#include "modules/common/proto/path_point.pb.h"

#include "modules/planning/common/decision_data.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/planning_data.h"
#include "modules/planning/common/planning_object.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class PlanningData {
 public:
  PlanningData() = default;

  // copy reference line to planning data
  void set_reference_line(std::unique_ptr<ReferenceLine>& reference_line);
  void set_decision_data(std::unique_ptr<DecisionData>& decision_data);
  void set_init_planning_point(const TrajectoryPoint& init_planning_point);
  void set_computed_trajectory(PublishableTrajectory computed_trajectory);

  const ReferenceLine& reference_line() const;
  const DecisionData& decision_data() const;
  const PublishableTrajectory& computed_trajectory() const;
  const TrajectoryPoint& init_planning_point() const;

  DecisionData* mutable_decision_data() const;
  PublishableTrajectory* mutable_computed_trajectory();

  virtual std::string type() const = 0;

  const PathData& path_data() const;
  const SpeedData& speed_data() const;

  PathData* mutable_path_data();
  SpeedData* mutable_speed_data();

  double timestamp() const;

 protected:
  std::unique_ptr<ReferenceLine> _reference_line;
  std::unique_ptr<DecisionData> _decision_data;
  PublishableTrajectory _computed_trajectory;
  TrajectoryPoint _init_planning_point;
  PathData _path_data;
  SpeedData _speed_data;
  double timestamp_ = 0.0;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_PLANNING_DATA_H_
