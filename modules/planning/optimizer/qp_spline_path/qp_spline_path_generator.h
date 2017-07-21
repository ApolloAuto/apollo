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

#ifndef BAIDU_IDG_HOUSTON_OPTIMIZER_QP_SPLINE_PATH_OPTIMIZER_QP_SPLINE_PATH_GENERATOR_H_
#define BAIDU_IDG_HOUSTON_OPTIMIZER_QP_SPLINE_PATH_OPTIMIZER_QP_SPLINE_PATH_GENERATOR_H_

#include "boost/property_tree/ptree.hpp"

#include "common/decision_data.h"
#include "common/environment.h"
#include "common/path/path_data.h"
#include "common/speed/speed_data.h"
#include "math/smoothing_spline/spline_1d_generator.h"
#include "optimizer/qp_spline_path_optimizer/qp_frenet_frame.h"
#include "optimizer/qp_spline_path_optimizer/qp_spline_path_configuration.h"

namespace apollo {
namespace planning {

class QPSplinePathGenerator {
 public:
  QPSplinePathGenerator(const boost::property_tree::ptree& property);

  bool generate(const Environment& environment,
                const ReferenceLine& reference_line,
                const DecisionData& decision_data, const SpeedData& speed_data,
                const ::adu::planning::TrajectoryPoint& init_point,
                PathData* const path_data);

 private:
  bool calculate_sl_point(const ReferenceLine& reference_line,
                          const ::adu::planning::TrajectoryPoint& traj_point,
                          FrenetFramePoint* const sl_point);

  bool init_coord_range(double* const start_s, double* const end_s);

  bool init_smoothing_spline(const Environment& environment,
                             const ReferenceLine& reference_line,
                             const double start_s, const double end_s);

  bool setup_constraint();

  bool setup_kernel();

  bool solve();

  bool extract(PathData* const path_data);

  // slightly shift generated trajectory due to sl - xy transformation error
  bool extract_init_coord_diff(double* const x_diff,
                               double* const y_diff) const;

 private:
  QPSplinePathConfiguration _qp_spline_path_configuration;
  FrenetFramePoint _init_point;
  std::unique_ptr<QpFrenetFrame> _qp_frenet_frame = nullptr;
  std::unique_ptr<Spline1dGenerator> _spline_generator = nullptr;
};
}  // namespace planning
}  // namespace apollo

#endif  // BAIDU_IDG_HOUSTON_OPTIMIZER_QP_SPLINE_PATH_OPTIMIZER_QP_SPLINE_PATH_GENERATOR_H_
