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
 * @file qp_spline_path_generator.h
 **/

#ifndef MODULES_PLANNING_OPTIMIZER_QP_SPLINE_PATH_QP_SPLINE_PATH_GENERATOR_H_
#define MODULES_PLANNING_OPTIMIZER_QP_SPLINE_PATH_QP_SPLINE_PATH_GENERATOR_H_

#include <memory>
#include <string>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/qp_spline_path_config.pb.h"

#include "modules/planning/common/decision_data.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/math/smoothing_spline/spline_1d_generator.h"
#include "modules/planning/optimizer/qp_spline_path/qp_frenet_frame.h"

namespace apollo {
namespace planning {

class QpSplinePathGenerator {
 public:
  QpSplinePathGenerator() = default;
  bool Init(const std::string& config_file);
  bool generate(const ReferenceLine& reference_line,
                const DecisionData& decision_data, const SpeedData& speed_data,
                const common::TrajectoryPoint& init_point,
                PathData* const path_data);

 private:
  bool calculate_sl_point(const ReferenceLine& reference_line,
                          const common::TrajectoryPoint& traj_point,
                          common::FrenetFramePoint* const sl_point);

  bool init_coord_range(const QpFrenetFrame& qp_frenet_frame,
                        double* const start_s, double* const end_s);

  bool init_smoothing_spline(const ReferenceLine& reference_line,
                             const common::FrenetFramePoint& init_frenet_point,
                             const double start_s, const double end_s);

  bool setup_constraint(const QpFrenetFrame& qp_frenet_frame);

  bool setup_kernel();

  bool solve();

  bool extract(PathData* const path_data);

  // slightly shift generated trajectory due to sl - xy transformation error
  bool extract_init_coord_diff(double* const x_diff,
                               double* const y_diff) const;

 private:
  QpSplinePathConfig qp_spline_path_config_;
  common::FrenetFramePoint init_point_;
  std::unique_ptr<Spline1dGenerator> spline_generator_;

  std::vector<double> knots_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_QP_SPLINE_PATH_QP_SPLINE_PATH_GENERATOR_H_
