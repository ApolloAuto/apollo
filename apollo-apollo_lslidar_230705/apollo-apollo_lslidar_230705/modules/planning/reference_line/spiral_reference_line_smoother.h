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

/*
 * spiral_reference_line_smoother.h
 */

#pragma once

#include <vector>

#include "Eigen/Dense"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_line_smoother.h"
#include "modules/planning/reference_line/reference_point.h"

namespace apollo {
namespace planning {

class SpiralReferenceLineSmoother : public ReferenceLineSmoother {
 public:
  explicit SpiralReferenceLineSmoother(
      const ReferenceLineSmootherConfig& config);

  virtual ~SpiralReferenceLineSmoother() = default;

  bool Smooth(const ReferenceLine& raw_reference_line,
              ReferenceLine* const smoothed_reference_line) override;

  // For offline navigation line smoothing
  int SmoothStandAlone(std::vector<Eigen::Vector2d> point2d,
                       std::vector<double>* ptr_theta,
                       std::vector<double>* ptr_kappa,
                       std::vector<double>* ptr_dkappa,
                       std::vector<double>* ptr_s, std::vector<double>* ptr_x,
                       std::vector<double>* ptr_y) const;

  void SetAnchorPoints(const std::vector<AnchorPoint>&) override;

  std::vector<common::PathPoint> Interpolate(const std::vector<double>& theta,
                                             const std::vector<double>& kappa,
                                             const std::vector<double>& dkappa,
                                             const std::vector<double>& s,
                                             const std::vector<double>& x,
                                             const std::vector<double>& y,
                                             const double resolution) const;

 private:
  bool Smooth(std::vector<Eigen::Vector2d> point2d,
              std::vector<double>* ptr_theta, std::vector<double>* ptr_kappa,
              std::vector<double>* ptr_dkappa, std::vector<double>* ptr_s,
              std::vector<double>* ptr_x, std::vector<double>* ptr_y) const;

 private:
  std::vector<common::PathPoint> Interpolate(
      const double start_x, const double start_y, const double start_s,
      const double theta0, const double kappa0, const double dkappa0,
      const double theta1, const double kappa1, const double dkappa1,
      const double delta_s, const double resolution) const;

  common::PathPoint to_path_point(const double x, const double y,
                                  const double s, const double theta,
                                  const double kappa,
                                  const double dkappa) const;

  std::vector<AnchorPoint> anchor_points_;

  bool fixed_start_point_ = false;

  double fixed_start_x_ = 0.0;

  double fixed_start_y_ = 0.0;

  double fixed_start_theta_ = 0.0;

  double fixed_start_kappa_ = 0.0;

  double fixed_start_dkappa_ = 0.0;

  double fixed_end_x_ = 0.0;

  double fixed_end_y_ = 0.0;

  double zero_x_ = 0.0;

  double zero_y_ = 0.0;
};

}  // namespace planning
}  // namespace apollo
