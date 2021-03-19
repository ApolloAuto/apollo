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

#include "modules/common/math/math_utils.h"
#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

using common::TrajectoryPoint;

double ValidationChecker::ProbabilityByCentripetalAcceleration(
    const LaneSequence& lane_sequence, const double speed) {
  double centripetal_acc_cost_sum = 0.0;
  double centripetal_acc_cost_sqr_sum = 0.0;
  for (int i = 0; i < lane_sequence.path_point_size(); ++i) {
    const auto& path_point = lane_sequence.path_point(i);
    double centripetal_acc = speed * speed * std::fabs(path_point.kappa());
    double centripetal_acc_cost =
        centripetal_acc / FLAGS_centripedal_acc_threshold;
    centripetal_acc_cost_sum += centripetal_acc_cost;
    centripetal_acc_cost_sqr_sum += centripetal_acc_cost * centripetal_acc_cost;
  }
  double mean_cost = centripetal_acc_cost_sqr_sum /
                     (centripetal_acc_cost_sum + FLAGS_double_precision);
  return std::exp(-FLAGS_centripetal_acc_coeff * mean_cost);
}

bool ValidationChecker::ValidCentripetalAcceleration(
    const std::vector<TrajectoryPoint>& trajectory_points) {
  for (size_t i = 0; i + 1 < trajectory_points.size(); ++i) {
    const auto& p0 = trajectory_points[i];
    const auto& p1 = trajectory_points[i + 1];
    double time_diff = std::abs(p1.relative_time() - p0.relative_time());
    if (time_diff < FLAGS_double_precision) {
      continue;
    }

    double theta_diff = std::abs(common::math::NormalizeAngle(
        p1.path_point().theta() - p0.path_point().theta()));
    double v = (p0.v() + p1.v()) * 0.5;
    double angular_a = v * theta_diff / time_diff;
    if (angular_a > FLAGS_centripedal_acc_threshold) {
      return false;
    }
  }
  return true;
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
