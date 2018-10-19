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

#include "modules/planning/lattice/trajectory_generation/lateral_qp_optimizer.h"

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

PiecewiseJerkTrajectory1d LateralQPOptimizer::GetOptimalTrajectory() const {
  CHECK(!opt_d_.empty() && !opt_d_prime_.empty() && !opt_d_pprime_.empty());

  PiecewiseJerkTrajectory1d optimal_trajectory(
      opt_d_.front(), opt_d_prime_.front(), opt_d_pprime_.front());

  for (size_t i = 1; i < opt_d_.size(); ++i) {
    double j = (opt_d_pprime_[i] - opt_d_pprime_[i - 1]) / delta_s_;
    optimal_trajectory.AppendSegment(j, delta_s_);
  }
  return optimal_trajectory;
}

std::vector<common::FrenetFramePoint> LateralQPOptimizer::GetFrenetFramePath()
    const {
  std::vector<common::FrenetFramePoint> frenet_frame_path;
  double accumulated_s = 0.0;
  for (size_t i = 0; i < opt_d_.size(); ++i) {
    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(accumulated_s);
    frenet_frame_point.set_l(opt_d_[i]);
    frenet_frame_point.set_dl(opt_d_prime_[i]);
    frenet_frame_point.set_ddl(opt_d_pprime_[i]);
    frenet_frame_path.push_back(std::move(frenet_frame_point));
    accumulated_s += delta_s_;
  }
  return frenet_frame_path;
}

}  // namespace planning
}  // namespace apollo
