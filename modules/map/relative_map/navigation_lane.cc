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

#include <algorithm>
#include <cmath>
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

using apollo::common::Path;
using apollo::common::VehicleStateProvider;
using apollo::common::math::Vec2d;
using apollo::common::util::DistanceXY;
using apollo::hdmap::Lane;
using apollo::common::util::operator+;
using apollo::perception::PerceptionObstacles;

NavigationLane::NavigationLane(const NavigationLaneConfig &config)
    : config_(config) {}

void NavigationLane::SetConfig(const NavigationLaneConfig &config) {
  config_ = config;
}

bool NavigationLane::GeneratePath() {
  navigation_path_list_.clear();
  current_navi_path_ = nullptr;

  // original_pose is in world coordination: ENU
  original_pose_ = VehicleStateProvider::instance()->original_pose();

  int navigation_line_num = navigation_info_.navigation_path_size();
  const auto &lane_marker = perception_obstacles_.lane_marker();

  auto generate_path_on_perception = [this, &lane_marker]() {
    current_navi_path_ = std::make_shared<NavigationPath>();
    auto *path = current_navi_path_->mutable_path();
    ConvertLaneMarkerToPath(lane_marker, path);
  };

  // priority: merge > navigation line > perception lane marker
  if (config_.lane_source() == NavigationLaneConfig::OFFLINE_GENERATED &&
      navigation_line_num > 0) {
    // Generate multiple navigation paths based on navigation lines.
    // Don't worry about efficiency because the total number of navigation lines
    // will not exceed 10 at most.
    for (int i = 0; i < navigation_line_num; ++i) {
      auto current_navi_path = std::make_shared<NavigationPath>();
      auto *path = current_navi_path->mutable_path();
      if (ConvertNavigationLineToPath(path, i)) {
        navigation_path_list_.emplace_back(i, current_navi_path);
      }
    }

    // If no navigation path is generated based on navigation lines, we generate
    // one where the vehicle is located based on perceived lane markers.
    if (navigation_path_list_.empty()) {
      generate_path_on_perception();
      return true;
    }

    // Sort navigation paths from left to right according to the vehicle's
    // direction.
    // In the FLU vehicle coordinate system, the y-coordinate on the left side
    // of the vehicle is positive, and the y-coordinate on the right side of the
    // vehicle is negative. The navigation paths can be sorted from left to
    // right according to the y-coordinate.
    navigation_path_list_.sort(
        [this](const NaviPathPair &left, const NaviPathPair &right) {
          double left_y = left.second->path().path_point(0).y();
          double right_y = right.second->path().path_point(0).y();
          return left_y > right_y;
        });

    // Get which navigation path the vehicle is currently on.
    double min_d = std::numeric_limits<double>::max();
    for (const auto &navi_path_pair : navigation_path_list_) {
      AINFO << "Current navigation path index is: " << navi_path_pair.first;
      double current_d = last_project_index_map_[navi_path_pair.first].second;
      if (current_d < min_d) {
        min_d = current_d;
        current_navi_path_ = navi_path_pair.second;
      }
    }

    // Merge current navigation path where the vehicle is located with perceived
    // lane markers.
    // Incorrect, don't use it temporarily.
    // auto *path = current_navi_path_->mutable_path();
    // MergeNavigationLineAndLaneMarker(path, current_line_index);
    return true;
  }

  // Generate a navigation path where the vehicle is located based on perceived
  // lane markers.
  generate_path_on_perception();
  return true;
}

double NavigationLane::EvaluateCubicPolynomial(const double c0, const double c1,
                                               const double c2, const double c3,
                                               const double z) const {
  return ((c3 * z + c2) * z + c1) * z + c0;
}

void NavigationLane::MergeNavigationLineAndLaneMarker(common::Path *path,
                                                      int line_index) {
  CHECK_NOTNULL(path);
  common::Path local_navi_path;
  common::Path &navigation_path = local_navi_path;

  // If "path" is non-empty, it indicates that a navigation path has been
  // generated based on a navigation line and does not need to be generated
  // again.
  if (path->path_point_size() > 0) {
    navigation_path = *path;
  } else {
    ConvertNavigationLineToPath(&navigation_path, line_index);
  }
  // If the size of current navigation path points is smaller than 2, just
  // generate a navigation path based on perceived lane markers.
  if (navigation_path.path_point_size() < 2) {
    ConvertLaneMarkerToPath(perception_obstacles_.lane_marker(), path);
    return;
  }

  common::Path lane_marker_path;
  ConvertLaneMarkerToPath(perception_obstacles_.lane_marker(),
                          &lane_marker_path);

  // If the size of lane marker path points is smaller than 2, merging is not
  // required.
  if (lane_marker_path.path_point_size() < 2) {
    return;
  }

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
    auto *p = path->add_path_point();
    const double kWeight = 0.9;
    *p = common::util::GetWeightedAverageOfTwoPathPoints(p1, p2, kWeight,
                                                         1 - kWeight);
  }
}

common::PathPoint NavigationLane::GetPathPointByS(const common::Path &path,
                                                  const int start_index,
                                                  const double s,
                                                  int *matched_index) {
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
  auto p = common::util::GetWeightedAverageOfTwoPathPoints(
      path.path_point(i), path.path_point(i + 1), 1 - r, r);
  return p;
}

bool NavigationLane::ConvertNavigationLineToPath(common::Path *path,
                                                 int line_index) {
  CHECK_NOTNULL(path);
  if (!navigation_info_.navigation_path(line_index).has_path() ||
      navigation_info_.navigation_path(line_index).path().path_point_size() ==
          0) {
    // path is empty
    return false;
  }
  path->set_name("Path from navigation line index " +
                 std::to_string(line_index));
  const auto &navigation_path =
      navigation_info_.navigation_path(line_index).path();
  auto proj_index_pair = UpdateProjectionIndex(navigation_path, line_index);
  // Can't find a proper projection index in the "line_index" lane according to
  // current vehicle position.
  int current_project_index = proj_index_pair.first;
  if (current_project_index < 0 ||
      current_project_index >= navigation_path.path_point_size()) {
    AINFO << "Invalid projection index " << current_project_index << " in line "
          << line_index;
    return false;
  } else {
    last_project_index_map_[line_index] = proj_index_pair;
  }

  double dist = navigation_path.path_point().rbegin()->s() -
                navigation_path.path_point(current_project_index).s();
  if (dist < 20) {
    return false;
  }

  // offset between the current vehicle state and navigation line
  const double dx = -original_pose_.position().x();
  const double dy = -original_pose_.position().y();
  const double ref_s = navigation_path.path_point(current_project_index).s();
  for (int i = std::max(0, current_project_index - 3);
       i < navigation_path.path_point_size(); ++i) {
    auto *point = path->add_path_point();
    point->CopyFrom(navigation_path.path_point(i));

    // shift to (0, 0)
    double enu_x = point->x() + dx;
    double enu_y = point->y() + dy;

    double flu_x = 0.0;
    double flu_y = 0.0;
    common::math::RotateAxis(original_pose_.heading(), enu_x, enu_y, &flu_x,
                             &flu_y);

    point->set_x(flu_x);
    point->set_y(flu_y);
    point->set_theta(common::math::NormalizeAngle(
        common::math::NormalizeAngle(point->theta()) -
        original_pose_.heading()));
    const double accumulated_s = navigation_path.path_point(i).s() - ref_s;
    point->set_s(accumulated_s);

    if (accumulated_s > FLAGS_max_len_from_navigation_line) {
      break;
    }
  }

  return true;
}

// project adc_state_ onto path
ProjIndexPair NavigationLane::UpdateProjectionIndex(const common::Path &path,
                                                    int line_index) {
  int index = 0;
  double min_d = std::numeric_limits<double>::max();
  const int path_size = path.path_point_size();
  int current_project_index = 0;
  auto item_iter = last_project_index_map_.find(line_index);
  if (item_iter != last_project_index_map_.end()) {
    current_project_index = std::max(0, item_iter->second.first);
  }

  if (FLAGS_enable_cyclic_rerouting) {
    // We create a condition here that sets the "current_project_index" to 0,
    // should the vehicle reach the end point of a cyclic/circular route. For
    // cyclic/circular navigation lines where the distance between their
    // starting and end points is very small, it is tedious and unnecessary to
    // re-send navigation lines every time. Because the starting point and the
    // end point cannot be completely consistent in a cyclic/circular navigaton
    // line. The vehicle's end point is usually beyond the starting point a
    // little when making a cyclic/circular navigation line. Therefore, the
    // "current_project_index" is reset to 0 if it is larger than 95% size of
    // the navigaton line and the vehicle's current position is near the
    // starting point of the navigatoin line.
    const int near_end_size = static_cast<int>(path_size * 0.95);
    if (current_project_index > near_end_size &&
        current_project_index < path_size) {
      if (DistanceXY(path.path_point(0),
                     path.path_point(current_project_index)) <
          FLAGS_max_distance_to_navigation_line) {
        min_d = DistanceXY(original_pose_.position(), path.path_point(0));
        if (min_d < FLAGS_max_distance_to_navigation_line) {
          return std::make_pair(0, min_d);
        }
      }
    }
  }

  for (int i = current_project_index; i + 1 < path_size; ++i) {
    const double d = DistanceXY(original_pose_.position(), path.path_point(i));
    if (d < min_d) {
      min_d = d;
      index = i;
    }
    const double kMaxDistance = 50.0;
    if (current_project_index != 0 && d > kMaxDistance) {
      break;
    }
  }

  if (min_d > FLAGS_max_distance_to_navigation_line) {
    return std::make_pair(-1, std::numeric_limits<double>::max());
  }
  return std::make_pair(index, min_d);
}

double NavigationLane::GetKappa(const double c1, const double c2,
                                const double c3, const double x) {
  const double dy = 3 * c3 * x * x + 2 * c2 * x + c1;
  const double d2y = 6 * c3 * x + 2 * c2;
  return std::fabs(d2y) / std::pow((1 + dy * dy), 1.5);
}

void NavigationLane::ConvertLaneMarkerToPath(
    const perception::LaneMarkers &lane_marker, common::Path *path) {
  CHECK_NOTNULL(path);

  path->set_name("Path from lane markers.");
  const auto &left_lane = lane_marker.left_lane_marker();
  const auto &right_lane = lane_marker.right_lane_marker();

  double path_c0 = (left_lane.c0_position() + right_lane.c0_position()) / 2.0;

  double left_quality = left_lane.quality() + 0.001;
  double right_quality = right_lane.quality() + 0.001;

  double quality_divider = left_quality + right_quality;

  double path_c1 = (left_lane.c1_heading_angle() * left_quality +
                    right_lane.c1_heading_angle() * right_quality) /
                   quality_divider;

  double path_c2 = (left_lane.c2_curvature() * left_quality +
                    right_lane.c2_curvature() * right_quality) /
                   quality_divider;

  double path_c3 = (left_lane.c3_curvature_derivative() * left_quality +
                    right_lane.c3_curvature_derivative() * right_quality) /
                   quality_divider;

  const double current_speed =
      VehicleStateProvider::instance()->vehicle_state().linear_velocity();
  double path_range = current_speed * FLAGS_ratio_navigation_lane_len_to_speed;
  if (path_range <= FLAGS_min_len_for_navigation_lane) {
    path_range = FLAGS_min_len_for_navigation_lane;
  } else {
    path_range = FLAGS_max_len_for_navigation_lane;
  }

  const double unit_z = 1.0;
  const double start_s = -2.0;
  double accumulated_s = start_s;
  for (double z = start_s; z <= path_range; z += unit_z) {
    double x1 = z;
    double y1 = 0;
    if (left_lane.view_range() > FLAGS_min_view_range_to_use_lane_marker ||
        right_lane.view_range() > FLAGS_min_view_range_to_use_lane_marker) {
      y1 = EvaluateCubicPolynomial(path_c0, path_c1, path_c2, path_c3, z);
    }
    auto *point = path->add_path_point();
    point->set_x(x1);
    point->set_y(y1);

    if (path->path_point_size() > 1) {
      auto &pre_point = path->path_point(path->path_point_size() - 2);
      accumulated_s += std::hypot(x1 - pre_point.x(), y1 - pre_point.y());
    }
    point->set_s(accumulated_s);
    point->set_theta(
        std::atan2(3 * path_c3 * x1 * x1 + 2 * path_c2 * x1 + path_c1, 1));
    point->set_kappa(GetKappa(path_c1, path_c2, path_c3, x1));

    const double k1 = GetKappa(path_c1, path_c2, path_c3, x1 - 0.0001);
    const double k2 = GetKappa(path_c1, path_c2, path_c3, x1 + 0.0001);
    point->set_dkappa((k2 - k1) / 0.0002);
  }

  left_width_ = (std::fabs(left_lane.c0_position()) +
                 std::fabs(right_lane.c0_position())) /
                2.0;
  right_width_ = left_width_;
}

bool NavigationLane::CreateMap(const MapGenerationParam &map_config,
                               MapMsg *map_msg) const {
  auto *navigation_info = map_msg->mutable_navigation_path();
  auto *hdmap = map_msg->mutable_hdmap();
  auto *lane_marker = map_msg->mutable_lane_marker();

  // A lambda expression for creating map.
  auto create_map_func = [&](const std::shared_ptr<NavigationPath> &navi_path,
                             bool generate_left_boundray = true) {
    const auto &path = navi_path->path();
    if (path.path_point_size() < 2) {
      AERROR << "The path length of line index is invalid";
      return false;
    }
    auto *lane = hdmap->add_lane();
    lane->mutable_id()->set_id(std::to_string(navi_path->path_priority()) +
                               "_" + path.name());
    (*navigation_info)[lane->id().id()] = *navi_path;
    // lane types
    lane->set_type(Lane::CITY_DRIVING);
    lane->set_turn(Lane::NO_TURN);

    // speed limit
    lane->set_speed_limit(map_config.default_speed_limit());

    // center line
    auto *curve_segment = lane->mutable_central_curve()->add_segment();
    curve_segment->set_heading(path.path_point(0).theta());
    auto *line_segment = curve_segment->mutable_line_segment();

    // left boundary
    hdmap::LineSegment *left_segment = nullptr;
    if (generate_left_boundray) {
      auto *left_boundary = lane->mutable_left_boundary();
      auto *left_boundary_type = left_boundary->add_boundary_type();
      left_boundary->set_virtual_(false);
      left_boundary_type->set_s(0.0);
      left_boundary_type->add_types(
          perception_obstacles_.lane_marker().left_lane_marker().lane_type());
      left_segment =
          left_boundary->mutable_curve()->add_segment()->mutable_line_segment();
    }

    // right boundary
    auto *right_boundary = lane->mutable_right_boundary();
    auto *right_boundary_type = right_boundary->add_boundary_type();
    right_boundary->set_virtual_(false);
    right_boundary_type->set_s(0.0);
    right_boundary_type->add_types(
        perception_obstacles_.lane_marker().right_lane_marker().lane_type());
    auto *right_segment =
        right_boundary->mutable_curve()->add_segment()->mutable_line_segment();

    const double lane_left_width =
        left_width_ <= 0.0
            ? map_config.default_left_width()
            : common::math::Clamp(left_width_, FLAGS_min_lane_half_width,
                                  FLAGS_max_lane_half_width);
    const double lane_right_width =
        right_width_ <= 0.0
            ? map_config.default_right_width()
            : common::math::Clamp(right_width_, FLAGS_min_lane_half_width,
                                  FLAGS_max_lane_half_width);

    for (const auto &path_point : path.path_point()) {
      auto *point = line_segment->add_point();
      point->set_x(path_point.x());
      point->set_y(path_point.y());
      point->set_z(path_point.z());
      if (generate_left_boundray) {
        auto *left_sample = lane->add_left_sample();
        left_sample->set_s(path_point.s());
        left_sample->set_width(lane_left_width);
        left_segment->add_point()->CopyFrom(
            *point + lane_left_width *
                         Vec2d::CreateUnitVec2d(path_point.theta() + M_PI_2));
      }

      auto *right_sample = lane->add_right_sample();
      right_sample->set_s(path_point.s());
      right_sample->set_width(lane_right_width);
      right_segment->add_point()->CopyFrom(
          *point + lane_right_width *
                       Vec2d::CreateUnitVec2d(path_point.theta() - M_PI_2));
    }
    return true;
  };

  lane_marker->CopyFrom(perception_obstacles_.lane_marker());

  // If no navigation path is generated based on navigation lines, we try to
  // create map with "current_navi_path_" which is generated based on perceived
  // lane markers.
  if (navigation_path_list_.empty()) {
    if (current_navi_path_) {
      return create_map_func(current_navi_path_);
    } else {
      return false;
    }
  }

  bool need_generate_left_boundray = true;
  int fail_num = 0;
  for (auto iter = navigation_path_list_.cbegin();
       iter != navigation_path_list_.cend(); ++iter) {
    std::size_t index = std::distance(navigation_path_list_.cbegin(), iter);
    if (!create_map_func(iter->second, need_generate_left_boundray)) {
      AWARN << "Failed to generate lane: " << index;
      need_generate_left_boundray = true;
      fail_num++;
      continue;
    }
    need_generate_left_boundray = (iter == navigation_path_list_.cbegin());

    // The left border of the middle lane uses the right border of the left
    // lane.
    int lane_index = index - fail_num;
    if (lane_index > 0) {
      auto *left_boundary =
          hdmap->mutable_lane(lane_index)->mutable_left_boundary();
      left_boundary->CopyFrom(hdmap->lane(lane_index - 1).right_boundary());
      auto *left_sample =
          hdmap->mutable_lane(lane_index)->mutable_left_sample();
      left_sample->CopyFrom(hdmap->lane(lane_index - 1).right_sample());
    }
  }

  // Set neighbor information for each lane
  int lane_num = hdmap->lane_size();
  if (lane_num < 2) {
    return true;
  }
  for (int i = 0; i < lane_num; ++i) {
    for (int j = i; j < lane_num - 1; ++j) {
      hdmap->mutable_lane(i)->add_right_neighbor_forward_lane_id()->CopyFrom(
          hdmap->lane(j + 1).id());
    }
    for (int k = i; k > 0; --k) {
      hdmap->mutable_lane(i)->add_left_neighbor_forward_lane_id()->CopyFrom(
          hdmap->lane(k - 1).id());
    }
  }

  return true;
}

}  // namespace relative_map
}  // namespace apollo
