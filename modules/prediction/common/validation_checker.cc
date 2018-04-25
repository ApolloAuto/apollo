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

#include "modules/prediction/common/validation_checker.h"

#include <algorithm>
#include <cmath>

#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

using ::apollo::common::PathPoint;
using ::apollo::common::TrajectoryPoint;

double ValidationChecker::ProbabilityByCentripedalAcceleration(
    const LaneSequence& lane_sequence, const double speed) {
  double centripetal_acc_cost_sum = 0.0;
  double centripetal_acc_cost_sqr_sum = 0.0;
  for (int i = 0; i < lane_sequence.path_point_size(); ++i) {
    const PathPoint& path_point = lane_sequence.path_point(i);
    double centripetal_acc = speed * speed * path_point.kappa();
    double centripetal_acc_cost =
        centripetal_acc / FLAGS_centripedal_acc_threshold;
    centripetal_acc_cost_sum += centripetal_acc_cost;
    centripetal_acc_cost_sqr_sum += centripetal_acc_cost * centripetal_acc_cost;
  }
  double mean_cost = centripetal_acc_cost_sqr_sum /
                     (centripetal_acc_cost_sum + FLAGS_double_precision);
  return std::exp(-FLAGS_centripetal_acc_coeff * mean_cost);
}

bool ValidationChecker::ValidCentripedalAcceleration(
    const std::vector<TrajectoryPoint>& trajectory_points) {
  std::size_t num_point = trajectory_points.size();
  if (num_point < 2) {
    return true;
  }
  double max_centripedal_acc = 0.0;
  for (std::size_t i = 0; i + 1 < num_point; ++i) {
    const auto& first_point = trajectory_points[i];
    const auto& second_point = trajectory_points[i + 1];
    double theta_diff = std::abs(second_point.path_point().theta() -
                                 first_point.path_point().theta());
    double time_diff =
        std::abs(second_point.relative_time() - first_point.relative_time());
    if (time_diff < FLAGS_double_precision) {
      continue;
    }
    double v = (first_point.v() + second_point.v()) / 2.0;
    double centripedal_acc = v * theta_diff / time_diff;
    max_centripedal_acc = std::max(max_centripedal_acc, centripedal_acc);
  }
  return max_centripedal_acc < FLAGS_centripedal_acc_threshold;
}

bool ValidationChecker::ValidTrajectoryPoint(
    const TrajectoryPoint& trajectory_point) {
  return trajectory_point.has_path_point() &&
         (!std::isnan(trajectory_point.path_point().x())) &&
         (!std::isnan(trajectory_point.path_point().y())) &&
         (!std::isnan(trajectory_point.path_point().theta())) &&
         (!std::isnan(trajectory_point.v())) &&
         (!std::isnan(trajectory_point.a())) &&
         (!std::isnan(trajectory_point.relative_time()));
}

}  // namespace prediction
}  // namespace apollo
