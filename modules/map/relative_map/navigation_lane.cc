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

#include "modules/map/proto/map_lane.pb.h"

#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/relative_map/common/relative_map_gflags.h"

namespace apollo {
namespace relative_map {

using apollo::common::VehicleStateProvider;
using apollo::common::math::Vec2d;
using apollo::common::util::DistanceXY;
using apollo::hdmap::Lane;
using apollo::common::util::operator+;
using apollo::perception::PerceptionObstacles;

NavigationLane::NavigationLane(const NavigationLaneConfig& config)
    : config_(config) {}

void NavigationLane::SetConfig(const NavigationLaneConfig& config) {
  config_ = config;
}

bool NavigationLane::GeneratePath() {
  // original_pose is in world coordination: ENU
  original_pose_ = VehicleStateProvider::instance()->original_pose();

  navigation_path_.Clear();
  auto* path = navigation_path_.mutable_path();
  const auto& lane_marker = perception_obstacles_.lane_marker();

  // priority: merge > navigation line > perception lane marker
  if (FLAGS_enable_navigation_line &&
      navigation_info_.navigation_path_size() > 0 &&
      std::fmin(lane_marker.left_lane_marker().quality(),
                lane_marker.right_lane_marker().quality()) >
          config_.min_lane_marker_quality()) {
    MergeNavigationLineAndLaneMarker(perception_obstacles_.lane_marker(), path);
  } else if (FLAGS_enable_navigation_line &&
             navigation_info_.navigation_path_size() > 0) {
    ConvertNavigationLineToPath(path);
  } else if (std::fmin(lane_marker.left_lane_marker().quality(),
                       lane_marker.right_lane_marker().quality()) >
             config_.min_lane_marker_quality()) {
    ConvertLaneMarkerToPath(perception_obstacles_.lane_marker(), path);
  } else {
    AERROR << "Navigation Path is empty because neither lane markers nor "
              "navigation line are available.";
  }

  return true;
}

double NavigationLane::EvaluateCubicPolynomial(const double c0, const double c1,
                                               const double c2, const double c3,
                                               const double z) const {
  return ((c3 * z + c2) * z + c1) * z + c0;
}

void NavigationLane::MergeNavigationLineAndLaneMarker(
    const perception::LaneMarkers& lane_marker, common::Path* path) {
  CHECK_NOTNULL(path);

  common::Path navigation_path;
  ConvertNavigationLineToPath(&navigation_path);

  common::Path lane_marker_path;
  ConvertLaneMarkerToPath(perception_obstacles_.lane_marker(),
                          &lane_marker_path);

  const double len = std::fmin(
      navigation_path.path_point(navigation_path.path_point_size() - 1).s(),
      lane_marker_path.path_point(lane_marker_path.path_point_size() - 1).s());

  const double ds = 1.0;
  int navigation_index = 0;
  int lane_marker_index = 0;
  for (double s = 0.0; s < len; s += ds) {
    auto p1 = GetPathPointByS(navigation_path, navigation_index, s,
                              &navigation_index);
    auto p2 = GetPathPointByS(lane_marker_path, lane_marker_index, s,
                              &lane_marker_index);
    auto* p = path->add_path_point();
    p->set_x((p1.x() + p2.x()) / 2.0);
    p->set_y((p1.y() + p2.y()) / 2.0);
    p->set_theta((p1.theta() + p2.theta()) / 2.0);
    p->set_kappa((p1.kappa() + p2.kappa()) / 2.0);
    p->set_dkappa((p1.dkappa() + p2.dkappa()) / 2.0);
  }
}

common::PathPoint NavigationLane::GetPathPointByS(const common::Path& path,
                                                  const int start_index,
                                                  const double s,
                                                  int* matched_index) {
  CHECK_NOTNULL(matched_index);

  constexpr double kEpsilon = 1e-9;
  if (std::fabs(path.path_point(start_index).s() - s) < kEpsilon) {
    *matched_index = start_index;
    return path.path_point(start_index);
  }
  int i = start_index;
  while (i + 1 < path.path_point_size() && path.path_point(i + 1).s() < s) {
    ++i;
  }
  *matched_index = i;

  // x, y, z, theta, kappa, s, dkappa, ddkappa
  const double r = (s - path.path_point(i).s()) /
                   (path.path_point(i + 1).s() - path.path_point(i).s());
  const double x =
      path.path_point(i).x() * (1 - r) + path.path_point(i + 1).x() * r;
  const double y =
      path.path_point(i).y() * (1 - r) + path.path_point(i + 1).y() * r;
  const double z =
      path.path_point(i).z() * (1 - r) + path.path_point(i + 1).z() * r;
  const double theta =
      path.path_point(i).theta() * (1 - r) + path.path_point(i + 1).theta() * r;
  const double kappa =
      path.path_point(i).kappa() * (1 - r) + path.path_point(i + 1).kappa() * r;
  const double dkappa = path.path_point(i).dkappa() * (1 - r) +
                        path.path_point(i + 1).dkappa() * r;
  const double ddkappa = path.path_point(i).ddkappa() * (1 - r) +
                         path.path_point(i + 1).ddkappa() * r;

  auto p = common::util::MakePathPoint(x, y, z, theta, kappa, dkappa, ddkappa);
  p.set_s(s);
  return p;
}

void NavigationLane::ConvertNavigationLineToPath(common::Path* path) {
  CHECK_NOTNULL(path);
  if (navigation_info_.navigation_path_size() == 0 ||
      !navigation_info_.navigation_path(0).has_path() ||
      navigation_info_.navigation_path(0).path().path_point_size() == 0) {
    // path is empty
    return;
  }
  path->set_name("Path from navigation.");
  UpdateProjectionIndex();

  // TODO(All): support multiple navigation path
  // currently, only 1 navigation path is supported
  const auto& navigation_path = navigation_info_.navigation_path(0).path();
  int curr_project_index = last_project_index_;

  // offset between the current vehicle state and navigation line
  const double dx = -original_pose_.position().x();
  const double dy = -original_pose_.position().y();
  for (int i = curr_project_index; i < navigation_path.path_point_size(); ++i) {
    auto* point = path->add_path_point();
    point->CopyFrom(navigation_path.path_point(i));

    // shift to (0, 0)
    double emu_x = point->x() + dx;
    double emu_y = point->y() + dy;

    double flu_x = 0.0;
    double flu_y = 0.0;
    common::math::RotateAxis(original_pose_.heading(), emu_x, emu_y, &flu_x,
                             &flu_y);

    point->set_x(flu_x);
    point->set_y(flu_y);
    const double accumulated_s =
        navigation_path.path_point(i).s() -
        navigation_path.path_point(curr_project_index).s();
    point->set_s(accumulated_s);

    if (accumulated_s > FLAGS_max_len_from_navigation_line) {
      break;
    }
  }

  // set left/right width invalid as no width info from navigation line
  left_width_ = -1.0;
  right_width_ = -1.0;
}

// project adc_state_ onto path
void NavigationLane::UpdateProjectionIndex() {
  // TODO(All): support multiple navigation path
  // currently, only 1 navigation path is supported
  const auto& path = navigation_info_.navigation_path(0).path();
  int index = 0;
  double min_d = std::numeric_limits<double>::max();
  for (int i = last_project_index_; i + 1 < path.path_point_size(); ++i) {
    const double d = DistanceXY(original_pose_.position(), path.path_point(i));
    if (d < min_d) {
      min_d = d;
      index = i;
    }
    const double kMaxDistance = 50.0;
    if (last_project_index_ != 0 && d > kMaxDistance) {
      break;
    }
  }
  last_project_index_ = index;
}

void NavigationLane::ConvertLaneMarkerToPath(
    const perception::LaneMarkers& lane_marker, common::Path* path) {
  CHECK_NOTNULL(path);

  path->set_name("Path from lane markers.");
  const auto& left_lane = lane_marker.left_lane_marker();
  const auto& right_lane = lane_marker.right_lane_marker();

  double path_c0 = (left_lane.c0_position() + right_lane.c0_position()) / 2.0;

  double quality_divider = left_lane.quality() + right_lane.quality();

  double path_c1 = (left_lane.c1_heading_angle() * left_lane.quality() +
      right_lane.c1_heading_angle() * left_lane.quality()) / quality_divider;

  double path_c2 = (left_lane.c2_curvature() * left_lane.quality() +
      right_lane.c2_curvature() * left_lane.quality()) / quality_divider;

  double path_c3 = (left_lane.c3_curvature_derivative() * left_lane.quality() +
      right_lane.c3_curvature_derivative() * left_lane.quality()) /
      quality_divider;

  double path_range = std::fmax(left_lane.view_range(),
                                right_lane.view_range());

  const double unit_z = 1.0;
  double accumulated_s = 0.0;
  for (double z = 0; z <= path_range; z += unit_z) {
    const double x_l = EvaluateCubicPolynomial(path_c0, path_c1,
                                               path_c2, path_c3, z);

    if (left_width_ < 0.0) {
      left_width_ = (std::fabs(left_lane.c0_position()) +
          std::fabs(right_lane.c0_position())) / 2.0;
    }
    if (right_width_ < 0.0) {
      right_width_ = left_width_;
    }

    double x1 = z;
    double y1 = x_l;

    auto* point = path->add_path_point();
    point->set_x(x1);
    point->set_y(y1);

    if (path->path_point_size() > 1) {
      auto& pre_point = path->path_point(path->path_point_size() - 2);
      accumulated_s += std::hypot(x1 - pre_point.x(), y1 - pre_point.y());
    }
    point->set_s(accumulated_s);
  }
}

bool NavigationLane::CreateMap(const MapGenerationParam& map_config,
                               MapMsg* map_msg) const {
  auto* navigation_info = map_msg->mutable_navigation_path();
  auto* hdmap = map_msg->mutable_hdmap();
  auto* lane_marker = map_msg->mutable_lane_marker();

  lane_marker->CopyFrom(perception_obstacles_.lane_marker());

  const auto& path = navigation_path_.path();
  if (path.path_point_size() < 2) {
    AERROR << "The path length is invalid";
    return false;
  }
  auto* lane = hdmap->add_lane();
  lane->mutable_id()->set_id(std::to_string(navigation_path_.path_priority()) +
                             "_" + path.name());
  (*navigation_info)[lane->id().id()] = navigation_path_;
  // lane types
  lane->set_type(Lane::CITY_DRIVING);
  lane->set_turn(Lane::NO_TURN);

  // speed limit
  lane->set_speed_limit(map_config.default_speed_limit());

  // center line
  auto* curve_segment = lane->mutable_central_curve()->add_segment();
  curve_segment->set_heading(path.path_point(0).theta());
  auto* line_segment = curve_segment->mutable_line_segment();
  // left boundary
  auto* left_boundary = lane->mutable_left_boundary();
  auto* left_boundary_type = left_boundary->add_boundary_type();
  left_boundary->set_virtual_(false);
  left_boundary_type->set_s(0.0);
  left_boundary_type->add_types(
      perception_obstacles_.lane_marker().left_lane_marker().lane_type());
  auto* left_segment =
      left_boundary->mutable_curve()->add_segment()->mutable_line_segment();
  // right boundary
  auto* right_boundary = lane->mutable_right_boundary();
  auto* right_boundary_type = right_boundary->add_boundary_type();
  right_boundary->set_virtual_(false);
  right_boundary_type->set_s(0.0);
  right_boundary_type->add_types(
      perception_obstacles_.lane_marker().right_lane_marker().lane_type());
  auto* right_segment =
      right_boundary->mutable_curve()->add_segment()->mutable_line_segment();
  const double lane_left_width =
      left_width_ > 0 ? left_width_ : map_config.default_left_width();
  const double lane_right_width =
      right_width_ > 0 ? right_width_ : map_config.default_right_width();
  for (const auto& path_point : path.path_point()) {
    auto* point = line_segment->add_point();
    point->set_x(path_point.x());
    point->set_y(path_point.y());
    point->set_z(path_point.z());
    auto* left_sample = lane->add_left_sample();
    left_sample->set_s(path_point.s());
    left_sample->set_width(lane_left_width);
    left_segment->add_point()->CopyFrom(
        *point +
        lane_left_width * Vec2d::CreateUnitVec2d(path_point.theta() + M_PI_2));
    auto* right_sample = lane->add_right_sample();
    right_sample->set_s(path_point.s());
    right_sample->set_width(lane_right_width);
    right_segment->add_point()->CopyFrom(
        *point +
        lane_right_width * Vec2d::CreateUnitVec2d(path_point.theta() - M_PI_2));
  }

  return true;
}

}  // namespace relative_map
}  // namespace apollo
