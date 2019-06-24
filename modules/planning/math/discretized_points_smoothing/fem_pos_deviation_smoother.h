/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @file
 **/

#pragma once

#include <utility>
#include <vector>

#include "modules/planning/proto/fem_pos_deviation_smoother_config.pb.h"

namespace apollo {
namespace planning {

/*
 * @brief:
 * This class solve an optimization problem:
 * Y
 * |
 * |                       P(x1, y1)  P(x2, y2)
 * |            P(x0, y0)                       ... P(x(k-1), y(k-1))
 * |P(start)
 * |
 * |________________________________________________________ X
 *
 *
 * Given an initial set of points from 0 to k-1,  The goal is to find a set of
 * points which makes the line P(start), P0, P(1) ... P(k-1) "smooth".
 */

class FemPosDeviationSmoother {
 public:
  explicit FemPosDeviationSmoother(const FemPosDeviationSmootherConfig& config);

  bool Solve(const std::vector<std::pair<double, double>>& raw_point2d,
             const std::vector<double>& bounds, std::vector<double>* opt_x,
             std::vector<double>* opt_y);

  bool SolveWithOsqp(const std::vector<std::pair<double, double>>& raw_point2d,
                     const std::vector<double>& bounds,
                     std::vector<double>* opt_x, std::vector<double>* opt_y);

  bool SolveWithIpopt(const std::vector<std::pair<double, double>>& raw_point2d,
                      const std::vector<double>& bounds,
                      std::vector<double>* opt_x, std::vector<double>* opt_y);

 private:
  FemPosDeviationSmootherConfig config_;
};
}  // namespace planning
}  // namespace apollo
