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

#pragma once


#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/lattice/trajectory_generation/lateral_qp_optimizer.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/qp_piecewise_jerk_path_config.pb.h"
#include "modules/planning/toolkits/optimizers/path_optimizer.h"

namespace apollo {
namespace planning {

class QpPiecewiseJerkPathOptimizer : public PathOptimizer {
 public:
  QpPiecewiseJerkPathOptimizer();
  bool Init(const ScenarioConfig::ScenarioTaskConfig& config) override;

 private:
  apollo::common::Status Process(const SpeedData& speed_data,
                                 const ReferenceLine& reference_line,
                                 const common::TrajectoryPoint& init_point,
                                 PathData* const path_data) override;

  std::vector<std::pair<double, double>> GetLateralBounds(
      const SLBoundary& adc_sl, const common::FrenetFramePoint& frenet_point,
      const double qp_delta_s, double path_length,
      const ReferenceLine& reference_line,
      const std::vector<const PathObstacle*>& obstacles);

 private:
  QpPiecewiseJerkPathConfig config_;
  std::unique_ptr<LateralQPOptimizer> lateral_qp_optimizer_;
};

}  // namespace planning
}  // namespace apollo
