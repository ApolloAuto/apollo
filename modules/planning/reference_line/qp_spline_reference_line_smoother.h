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
 * @file qp_spline_reference_line_smoother.h
 **/

#ifndef MODULES_PLANNING_REFERENCE_LINE_QP_SPLINE_REFERENCE_LINE_SMOOTHER_H_
#define MODULES_PLANNING_REFERENCE_LINE_QP_SPLINE_REFERENCE_LINE_SMOOTHER_H_

#include <memory>
#include <string>
#include <vector>

#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/qp_spline_reference_line_smoother_config.pb.h"

#include "modules/planning/math/smoothing_spline/spline_2d_solver.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_line_smoother.h"
#include "modules/planning/reference_line/reference_point.h"

namespace apollo {
namespace planning {

class QpSplineReferenceLineSmoother : ReferenceLineSmoother {
 public:
  QpSplineReferenceLineSmoother() = default;
  virtual ~QpSplineReferenceLineSmoother() = default;

  void Init(const std::string& config_file,
            Spline2dSolver* const spline_solver) override;

  void Init(const QpSplineReferenceLineSmootherConfig& refline_smooth_config,
            Spline2dSolver* const spline_solver) override;

  bool Smooth(const ReferenceLine& raw_reference_line,
              ReferenceLine* const smoothed_reference_line) override;

 private:
  void Clear();

  bool Sampling(const ReferenceLine& raw_reference_line);

  bool ApplyConstraint(const ReferenceLine& raw_reference_line);

  bool ApplyKernel();

  bool Solve();

  bool ExtractEvaluatedPoints(
      const ReferenceLine& raw_reference_line, const std::vector<double>& vec_t,
      std::vector<common::PathPoint>* const path_points) const;

  bool GetSFromParamT(const double t, double* const s) const;

  std::uint32_t FindIndex(const double t) const;

 private:
  QpSplineReferenceLineSmootherConfig smoother_config_;
  std::vector<double> t_knots_;
  std::vector<common::PathPoint> ref_points_;
  Spline2dSolver* spline_solver_ = nullptr;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_REFERENCE_LINE_QP_SPLINE_REFERENCE_LINE_SMOOTHER_H_
