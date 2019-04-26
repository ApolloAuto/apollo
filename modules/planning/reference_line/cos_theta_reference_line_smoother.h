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

#include "Eigen/Dense"

#include "modules/planning/math/curve_math.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_line_smoother.h"
#include "modules/planning/reference_line/reference_point.h"

namespace apollo {
namespace planning {

class CosThetaReferenceLineSmoother : public ReferenceLineSmoother {
 public:
  explicit CosThetaReferenceLineSmoother(
      const ReferenceLineSmootherConfig& config);

  virtual ~CosThetaReferenceLineSmoother() = default;

  bool Smooth(const ReferenceLine& raw_reference_line,
              ReferenceLine* const smoothed_reference_line) override;

  void SetAnchorPoints(const std::vector<AnchorPoint>&) override;

 private:
  bool CosThetaSmooth(const std::vector<Eigen::Vector2d>& point2d,
                      const std::vector<double>& lateral_bounds,
                      std::vector<Eigen::Vector2d>* ptr_smoothed_point2d);

  void NormalizePoints(std::vector<Eigen::Vector2d>* xy_points);

  void DeNormalizePoints(std::vector<Eigen::Vector2d>* xy_points);

  bool GenerateRefPointProfile(const ReferenceLine& raw_reference_line,
                               const std::vector<Eigen::Vector2d>& xy_points,
                               std::vector<ReferencePoint>* reference_points);

  std::vector<AnchorPoint> anchor_points_;

  double max_point_deviation_ = 0.1;

  size_t num_of_iterations_ = 3000;

  bool has_start_point_constraint_ = false;

  bool has_end_point_constraint_ = false;

  double start_x_derivative_ = 0.0;

  double start_y_derivative_ = 0.0;

  double weight_cos_included_angle_ = 0.0;

  double acceptable_tol_ = 1e-5;

  double relax_ = 0.2;

  double zero_x_ = 0.0;

  double zero_y_ = 0.0;

  std::vector<std::string> ipopt_failure_status_ = {
      "Solve_Succeeded",
      "Solved_To_Acceptable_Level",
      "Infeasible_Problem_Detected",
      "Search_Direction_Becomes_Too_Small",
      "Diverging_Iterates",
      "User_Requested_Stop",
      "Feasible_Point_Found",
      "Maximum_Iterations_Exceeded",
      "Restoration_Failed",
      "Error_In_Step_Computation",
      "Not_Enough_Degrees_Of_Freedom",
      "Invalid_Problem_Definition",
      "Invalid_Option",
      "Invalid_Number_Detected",
      "Unrecoverable_Exception",
      "NonIpopt_Exception_Thrown"
      "Insufficient_Memory",
      "Internal_Error"};
};

}  // namespace planning
}  // namespace apollo
