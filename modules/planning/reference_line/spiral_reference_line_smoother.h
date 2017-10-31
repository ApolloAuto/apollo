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

#ifndef MODULES_PLANNING_REFERENCE_LINE_SPIRAL_REFERENCE_LINE_SMOOTHER_H_
#define MODULES_PLANNING_REFERENCE_LINE_SPIRAL_REFERENCE_LINE_SMOOTHER_H_

#include <vector>

#include "Eigen/Dense"

#include "modules/planning/proto/planning.pb.h"

#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_line_smoother.h"
#include "modules/planning/reference_line/reference_point.h"

namespace apollo {
namespace planning {

class SpiralReferenceLineSmoother : public ReferenceLineSmoother {
 public:
  explicit SpiralReferenceLineSmoother(
      const double max_point_deviation_distance);
  virtual ~SpiralReferenceLineSmoother() = default;

  bool Smooth(const ReferenceLine& raw_reference_line,
              ReferenceLine* const smoothed_reference_line) override;

 private:
  bool Smooth(std::vector<Eigen::Vector2d> point2d,
              std::vector<common::PathPoint>* ptr_smoothed_point2d) const;

  std::vector<common::PathPoint> to_path_points(
      const double start_x, const double start_y, const double start_s,
      const double theta0, const double kappa0, const double dkappa0,
      const double theta1, const double kappa1, const double dkappa1,
      const double delta_s, const double resolution) const;

  common::PathPoint to_path_point(const double x, const double y,
                                  const double s, const double theta,
                                  const double kappa,
                                  const double dkappa) const;

  double max_point_deviation_ = 0.0;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_REFERENCE_LINE_SPIRAL_REFERENCE_LINE_SMOOTHER_H_
