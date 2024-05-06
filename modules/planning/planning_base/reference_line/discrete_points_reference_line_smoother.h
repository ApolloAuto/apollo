/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <utility>
#include <vector>

#include "modules/planning/planning_base/proto/reference_line_smoother_config.pb.h"

#include "modules/planning/planning_base/reference_line/reference_line.h"
#include "modules/planning/planning_base/reference_line/reference_line_smoother.h"
#include "modules/planning/planning_base/reference_line/reference_point.h"

namespace apollo {
namespace planning {

class DiscretePointsReferenceLineSmoother : public ReferenceLineSmoother {
 public:
  explicit DiscretePointsReferenceLineSmoother(
      const ReferenceLineSmootherConfig& config);

  virtual ~DiscretePointsReferenceLineSmoother() = default;

  bool Smooth(const ReferenceLine& raw_reference_line,
              ReferenceLine* const smoothed_reference_line) override;

  void SetAnchorPoints(const std::vector<AnchorPoint>&) override;

 private:
  bool CosThetaSmooth(
      const std::vector<std::pair<double, double>>& raw_point2d,
      const std::vector<double>& bounds,
      std::vector<std::pair<double, double>>* ptr_smoothed_point2d);

  bool FemPosSmooth(
      const std::vector<std::pair<double, double>>& raw_point2d,
      const std::vector<double>& bounds,
      std::vector<std::pair<double, double>>* ptr_smoothed_point2d);

  void NormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  void DeNormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  bool GenerateRefPointProfile(
      const ReferenceLine& raw_reference_line,
      const std::vector<std::pair<double, double>>& xy_points,
      std::vector<ReferencePoint>* reference_points);

  std::vector<AnchorPoint> anchor_points_;

  double zero_x_ = 0.0;

  double zero_y_ = 0.0;
};

}  // namespace planning
}  // namespace apollo
