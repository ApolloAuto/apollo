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

#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_line_smoother.h"
#include "modules/planning/reference_line/reference_point.h"

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
      const std::vector<std::pair<double, double>>& point2d,
      const std::vector<double>& lateral_bounds,
      std::vector<std::pair<double, double>>* ptr_smoothed_point2d);

  bool FemPosSmooth(
      const std::vector<std::pair<double, double>>& point2d,
      const std::vector<double>& lateral_bounds,
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

  // cos_theta smoothing parameters
  bool use_cos_theta_ = false;

  double weight_cos_included_angle_ = 0.0;

  double weight_anchor_points_ = 0.0;

  double weight_length_ = 0.0;

  size_t print_level_ = 0;

  size_t max_num_of_iterations_ = 500;

  size_t acceptable_num_of_iterations_ = 15;

  double tol_ = 1e-8;

  double acceptable_tol_ = 1e-6;

  bool use_automatic_differentiation_ = false;

  // fem_pos smoothing parameters
  bool use_fem_pos_ = false;

  double weight_fem_pos_deviation_ = 0.0;

  double weight_ref_deviation_ = 0.0;

  double weight_path_length_ = 0.0;

  size_t max_iter_ = 0;

  double time_limit_ = 0.0;

  bool verbose_ = false;

  bool scaled_termination_ = false;

  bool warm_start_ = false;
};

}  // namespace planning
}  // namespace apollo
