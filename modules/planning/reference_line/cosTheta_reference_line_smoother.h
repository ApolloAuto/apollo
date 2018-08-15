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

#ifndef MODULES_PLANNING_REFERENCE_LINE_COSTHETA_REFERENCE_LINE_SMOOTHER_H_
#define MODULES_PLANNING_REFERENCE_LINE_COSTHETA_REFERENCE_LINE_SMOOTHER_H_

#include <utility>
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
  bool Smooth(std::vector<Eigen::Vector2d> point2d,
              std::vector<common::PathPoint>* ptr_interpolated_point2d,
              std::vector<double> lateral_bounds);

  common::PathPoint to_path_point(const double x, const double y,
                                  const double x_derivative,
                                  const double y_derivative) const;

  common::PathPoint to_path_point(const double* point_info) const;

  void quintic_hermite_point(const double t,
                             const common::PathPoint front_point,
                             const common::PathPoint back_point,
                             double* quintic_hermite_point_info);

  double quintic_hermite_s(const double t, common::PathPoint front_point,
                           common::PathPoint back_point);

  double arclength_integration(const double t, common::PathPoint front_point,
                               common::PathPoint back_point);

  std::vector<AnchorPoint> anchor_points_;

  double max_point_deviation_ = 0.1;

  std::size_t num_of_iterations_ = 3000;

  bool has_start_point_constraint_ = false;

  bool has_end_point_constraint_ = false;

  double start_x_derivative_ = 0.0;

  double start_x_2nd_derivative_ = 0.0;

  double start_y_derivative_ = 0.0;

  double start_y_2nd_derivative_ = 0.0;

  double weight_cos_included_angle_ = 0.0;

  double acceptable_tol_ = 1e-5;

  double resolution_ = 0.0;

  double relax_ = 0.2;

  double kappa_filter_ = 1.0e9;

  double dkappa_filter_ = 1.0e9;

  std::size_t density_ = 0;

  double zero_x_ = 0.0;

  double zero_y_ = 0.0;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_REFERENCE_LINE_COSTHETA_REFERENCE_LINE_SMOOTHER_H_
