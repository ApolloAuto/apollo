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

#ifndef MODULES_PLANNING_OPTIMIZER_QP_SPLINE_ST_QP_SPLINE_ST_SPEED_OPTIMIZER_H_
#define MODULES_PLANNING_OPTIMIZER_QP_SPLINE_ST_QP_SPLINE_ST_SPEED_OPTIMIZER_H_

#include "modules/planning/optimizer/speed_optimizer.h"

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/planning/proto/qp_spline_st_speed_config.pb.h"

#include "modules/map/hdmap/hdmap.h"

namespace apollo {
namespace planning {

class QpSplineStSpeedOptimizer : public SpeedOptimizer {
 public:
  explicit QpSplineStSpeedOptimizer(const std::string& name);

 private:
  virtual common::Status process(
      const PathData& path_data,
      const apollo::common::TrajectoryPoint& init_point,
      DecisionData* const decision_data, SpeedData* const speed_data) const;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_QP_SPLINE_ST_QP_SPLINE_ST_SPEED_OPTIMIZER_H_
