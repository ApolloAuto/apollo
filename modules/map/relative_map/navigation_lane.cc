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

#include "modules/map/relative_map/navigation_lane.h"

#include <limits>

#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"

namespace apollo {
namespace relative_map {
namespace {

double GetDistance(const common::VehicleState& adc_state,
                   const common::PathPoint& p) {
  return std::hypot(adc_state.x() - p.x(), adc_state.y() - p.y());
}
}
using apollo::perception::PerceptionObstacles;
using apollo::common::VehicleStateProvider;

bool NavigationLane::Update(const PerceptionObstacles& perception_obstacles) {
  // udpate perception_obstacles_
  perception_obstacles_ = perception_obstacles;
  if (!perception_obstacles.has_lane_marker()) {
    AERROR << "No lane marker in perception_obstacles.";
    return false;
  }

  // update adc_state_ from VehicleStateProvider
  adc_state_ = VehicleStateProvider::instance()->vehicle_state();

  navigation_path_.Clear();
  auto* path = navigation_path_.mutable_path();
  const auto& lane_marker = perception_obstacles_.lane_marker();

  const double kQualityThreshold = 0.5;
  if (navigation_info_.navigation_path_size() > 0 &&
      std::fmax(lane_marker.left_lane_marker().quality(),
                lane_marker.right_lane_marker().quality()) <
          kQualityThreshold) {
    ConvertNavigationLineToPath();
  } else {
    ConvertLaneMarkerToPath(perception_obstacles_.lane_marker(), path);
  }
  return true;
}

double NavigationLane::EvaluateCubicPolynomial(const double c0, const double c1,
                                               const double c2, const double c3,
                                               const double z) const {
  return c3 * std::pow(z, 3) + c2 * std::pow(z, 2) + c1 * z + c0;
}

void NavigationLane::ConvertNavigationLineToPath() {
  if (navigation_info_.navigation_path_size() == 0 ||
      !navigation_info_.navigation_path(0).has_path() ||
      navigation_info_.navigation_path(0).path().path_point_size() == 0) {
    // path is empty
    return;
  }
  navigation_path_.Clear();
  auto* curr_path = navigation_path_.mutable_path();
  UpdateProjectionIndex();

  // TODO(All): support multiple navigation path
  // only support 1 navigation path
  const auto& path = navigation_info_.navigation_path(0).path();
  int curr_project_index = last_project_index_;
  for (int i = curr_project_index; i < path.path_point_size(); ++i) {
    auto* point = curr_path->add_path_point();
    point->CopyFrom(path.path_point(i));
    const double accumulated_s =
        path.path_point(i).s() - path.path_point(curr_project_index).s();
    point->set_s(accumulated_s);
  }
}

// project adc_state_ onto path
void NavigationLane::UpdateProjectionIndex() {
  // TODO(All): support multiple navigation path
  // only support 1 navigation path
  const auto& path = navigation_info_.navigation_path(0).path();
  int index = 0;
  double min_d = std::numeric_limits<double>::max();
  for (int i = last_project_index_; i + 1 < path.path_point_size(); ++i) {
    const double d = GetDistance(adc_state_, path.path_point(i));
    if (d < min_d) {
      min_d = d;
      index = i;
    }
    const double kMaxDistance = 50.0;
    if (d > kMaxDistance) {
      break;
    }
  }
  last_project_index_ = index;
}

void NavigationLane::ConvertLaneMarkerToPath(
    const perception::LaneMarkers& lane_marker, common::Path* path) {
  const auto& left_lane = lane_marker.left_lane_marker();
  const auto& right_lane = lane_marker.right_lane_marker();

  const double unit_z = 1.0;
  double accumulated_s = 0.0;
  for (double z = 0;
       z <= std::fmin(left_lane.view_range(), right_lane.view_range());
       z += unit_z) {
    const double x_l = EvaluateCubicPolynomial(
        left_lane.c0_position(), left_lane.c1_heading_angle(),
        left_lane.c2_curvature(), left_lane.c3_curvature_derivative(), z);
    const double x_r = EvaluateCubicPolynomial(
        right_lane.c0_position(), right_lane.c1_heading_angle(),
        right_lane.c2_curvature(), right_lane.c3_curvature_derivative(), z);

    double x1 = 0.0;
    double y1 = 0.0;
    // rotate from vehicle axis to x-y axis
    common::math::RotateAxis(-adc_state_.heading(), z, (x_l + x_r) / 2.0, &x1,
                             &y1);

    // shift to get point on x-y axis
    x1 += adc_state_.x();
    y1 += adc_state_.y();

    auto* point = path->add_path_point();
    point->set_x(x1);
    point->set_y(y1);
    point->set_s(accumulated_s);
    accumulated_s += std::hypot(x1, y1);
  }
}

}  // namespace relative_map
}  // namespace apollo
