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
 * @file qp_spline_path_optimizer.cpp
 **/
#include "optimizer/qp_spline_path_optimizer.h"

namespace apollo {
namespace planning {

QPSplinePathOptimizer::QPSplinePathOptimizer(
    const std::string& name, const boost::property_tree::ptree& ptree)
    : PathOptimizer(name) {
  _path_generator.reset(new QPSplinePathGenerator(ptree));
}

ErrorCode QPSplinePathOptimizer::optimize(
    const DataCenter& data_center, const SpeedData& speed_data,
    const ReferenceLine& reference_line,
    const ::adu::planning::TrajectoryPoint& init_point,
    DecisionData* const decision_data, PathData* const path_data) const {
  const Environment& env = data_center.current_frame()->environment();
  QUIT_IF(!_path_generator->generate(env, reference_line, *decision_data,
                                     speed_data, init_point, path_data),
          ErrorCode::PLANNING_ERROR_FAILED, Level::ERROR,
          "failed to generate spline path!");
  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace apollo
