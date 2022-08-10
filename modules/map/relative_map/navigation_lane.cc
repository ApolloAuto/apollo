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

/**
 * @file
 * @brief This file provides the implementation of the class `NavigationLane`.
 */

#include "modules/map/relative_map/navigation_lane.h"

#include <algorithm>
#include <limits>
#include <string>

#include "absl/strings/str_cat.h"
#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/util.h"
#include "modules/common_msgs/map_msgs/map_lane.pb.h"
#include "modules/map/relative_map/common/relative_map_gflags.h"

namespace apollo {
namespace relative_map {

using apollo::common::VehicleStateProvider;
using apollo::common::math::Vec2d;
using apollo::common::util::DistanceXY;
using apollo::hdmap::Lane;
using apollo::common::util::operator+;
using apollo::common::util::IsFloatEqual;

namespace {
/**
 * @brief Create a single lane map.
 * @param navi_path_tuple A navigation path tuple.
 * @param map_config Map generation configuration information.
 * @param perception_obstacles The Perceived obstacle information and the lane
 * markings are used here.
 * @param hdmap The output single lane map in high-definition map format in the
 * relative map.
 * @param navigation_path The output navigation path map in the relative map.
 * @return True if the map is created; false otherwise.
 */
bool CreateSingleLaneMap(
    const NaviPathTuple &navi_path_tuple, const MapGenerationParam &map_config,
    const perception::PerceptionObstacles &perception_obstacles,
    hdmap::Map *const hdmap,
    google::protobuf::Map<std::string, NavigationPath> *const navigation_path) {
  CHECK_NOTNULL(hdmap);
  CHECK_NOTNULL(navigation_path);

  const auto &navi_path = std::get<3>(navi_path_tuple);
  const auto &path = navi_path->path();
  if (path.path_point_size() < 2) {
    AERROR << "The path length of line index is invalid";
    return false;
  }
  auto *lane = hdmap->add_lane();
  lane->mutable_id()->set_id(
      absl::StrCat(navi_path->path_priority(), "_", path.name()));
  (*navigation_path)[lane->id().id()] = *navi_path;
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
  if (FLAGS_relative_map_generate_left_boundray) {
    auto *left_boundary = lane->mutable_left_boundary();
    auto *left_boundary_type = left_boundary->add_boundary_type();
    left_boundary->set_virtual_(false);
    left_boundary_type->set_s(0.0);
    left_boundary_type->add_types(
        perception_obstacles.lane_marker().left_lane_marker().lane_type());
    left_segment =
        left_boundary->mutable_curve()->add_segment()->mutable_line_segment();
  }

  // right boundary
  auto *right_boundary = lane->mutable_right_boundary();
  auto *right_boundary_type = right_boundary->add_boundary_type();
  right_boundary->set_virtual_(false);
  right_boundary_type->set_s(0.0);
  right_boundary_type->add_types(
      perception_obstacles.lane_marker().right_lane_marker().lane_type());
  auto *right_segment =
      right_boundary->mutable_curve()->add_segment()->mutable_line_segment();

  const double lane_left_width = std::get<1>(navi_path_tuple);
  const double lane_right_width = std::get<2>(navi_path_tuple);

  for (const auto &path_point : path.path_point()) {
    auto *point = line_segment->add_point();
    point->set_x(path_point.x());
    point->set_y(path_point.y());
    point->set_z(path_point.z());
    if (FLAGS_relative_map_generate_left_boundray) {
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
        *point +
        lane_right_width * Vec2d::CreateUnitVec2d(path_point.theta() - M_PI_2));
  }
  return true;
}
}  // namespace

NavigationLane::NavigationLane(const NavigationLaneConfig &config)
    : config_(config) {}

void NavigationLane::SetConfig(const NavigationLaneConfig &config) {
  config_ = config;
}

void NavigationLane::SetVehicleStateProvider(
    common::VehicleStateProvider *vehicle_state_provider) {
  vehicle_state_provider_ = vehicle_state_provider;
}

void NavigationLane::UpdateNavigationInfo(
    const NavigationInfo &navigation_path) {
  navigation_info_ = navigation_path;
  last_project_index_map_.clear();
  navigation_path_list_.clear();
  current_navi_path_tuple_ = std::make_tuple(-1, -1.0, -1.0, nullptr);
  if (FLAGS_enable_cyclic_rerouting) {
    UpdateStitchIndexInfo();
  }
}

bool NavigationLane::GeneratePath() {
  navigation_path_list_.clear();
  current_navi_path_tuple_ = std::make_tuple(-1, -1.0, -1.0, nullptr);

  // original_pose is in world coordination: ENU
  original_pose_ = vehicle_state_provider_->original_pose();

  int navigation_line_num = navigation_info_.navigation_path_size();
  const auto &lane_marker = perception_obstacles_.lane_marker();

  auto generate_path_on_perception_func = [this, &lane_marker]() {
    auto current_navi_path = std::make_shared<NavigationPath>();
    auto *path = current_navi_path->mutable_path();
    ConvertLaneMarkerToPath(lane_marker, path);
    current_navi_path->set_path_priority(0);
    double left_width = perceived_left_width_ > 0.0 ? perceived_left_width_
                                                    : default_left_width_;
    double right_width = perceived_right_width_ > 0.0 ? perceived_right_width_
                                                      : default_right_width_;
    current_navi_path_tuple_ =
        std::make_tuple(0, left_width, right_width, current_navi_path);
  };

  ADEBUG << "Beginning of NavigationLane::GeneratePath().";
  ADEBUG << "navigation_line_num: " << navigation_line_num;

  // priority: merge > navigation line > perception lane marker
  if (config_.lane_source() == NavigationLaneConfig::OFFLINE_GENERATED &&
      navigation_line_num > 0) {
    // Generate multiple navigation paths based on navigation lines.
    // Don't worry about efficiency because the total number of navigation lines
    // will not exceed 10 at most.
    for (int i = 0; i < navigation_line_num; ++i) {
      auto current_navi_path = std::make_shared<NavigationPath>();
      auto *path = current_navi_path->mutable_path();
      if (ConvertNavigationLineToPath(i, path)) {
        current_navi_path->set_path_priority(
            navigation_info_.navigation_path(i).path_priority());
        navigation_path_list_.emplace_back(
            i, default_left_width_, default_right_width_, current_navi_path);
      }
    }

    // If no navigation path is generated based on navigation lines, we generate
    // one where the vehicle is located based on perceived lane markers.
    if (navigation_path_list_.empty()) {
      generate_path_on_perception_func();
      return true;
    }

    // Sort navigation paths from left to right according to the vehicle's
    // direction.
    // In the FLU vehicle coordinate system, the y-coordinate on the left side
    // of the vehicle is positive, and the right value is negative. Therefore,
    // the navigation paths can be sorted from left to right according to its
    // y-coordinate.
    navigation_path_list_.sort(
        [](const NaviPathTuple &left, const NaviPathTuple &right) {
          double left_y = std::get<3>(left)->path().path_point(0).y();
          double right_y = std::get<3>(right)->path().path_point(0).y();
          return left_y > right_y;
        });

    // Get which navigation path the vehicle is currently on.
    double min_d = std::numeric_limits<double>::max();
    for (const auto &navi_path_tuple : navigation_path_list_) {
      int current_line_index = std::get<0>(navi_path_tuple);
      ADEBUG << "Current navigation path index is: " << current_line_index;
      double current_d = std::numeric_limits<double>::max();
      auto item_iter = last_project_index_map_.find(current_line_index);
      if (item_iter != last_project_index_map_.end()) {
        current_d = item_iter->second.second;
      }
      if (current_d < min_d) {
        min_d = current_d;
        current_navi_path_tuple_ = navi_path_tuple;
      }
    }

    // Merge current navigation path where the vehicle is located with perceived
    // lane markers.
    auto *path = std::get<3>(current_navi_path_tuple_)->mutable_path();
    MergeNavigationLineAndLaneMarker(std::get<0>(current_navi_path_tuple_),
                                     path);

    // Set the width for the navigation path which the vehicle is currently on.
    double left_width = perceived_left_width_ > 0.0 ? perceived_left_width_
                                                    : default_left_width_;
    double right_width = perceived_right_width_ > 0.0 ? perceived_right_width_
                                                      : default_right_width_;
    if (!IsFloatEqual(left_width, default_left_width_) &&
        !IsFloatEqual(right_width, default_right_width_)) {
      left_width = left_width > default_left_width_ ? left_width - min_d
                                                    : left_width + min_d;
      right_width = right_width > default_right_width_ ? right_width - min_d
                                                       : right_width + min_d;
    }

    ADEBUG << "The left width of current lane is: " << left_width
           << " and the right width of current lane is: " << right_width;

    std::get<1>(current_navi_path_tuple_) = left_width;
    std::get<2>(current_navi_path_tuple_) = right_width;
    auto curr_navi_path_iter = std::find_if(
        std::begin(navigation_path_list_), std::end(navigation_path_list_),
        [this](const NaviPathTuple &item) {
          return std::get<0>(item) == std::get<0>(current_navi_path_tuple_);
        });
    if (curr_navi_path_iter != std::end(navigation_path_list_)) {
      std::get<1>(*curr_navi_path_iter) = left_width;
      std::get<2>(*curr_navi_path_iter) = right_width;
    }

    // Set the width between each navigation path and its adjacent path.
    // The reason for using average of multiple points is to prevent too much
    // interference from a singularity.
    // If current navigation path is the path which the vehicle is currently
    // on, the current lane width uses the perceived width.
    int average_point_size = 5;
    for (auto iter = navigation_path_list_.begin();
         iter != navigation_path_list_.end(); ++iter) {
      const auto &curr_path = std::get<3>(*iter)->path();

      // Left neighbor
      auto prev_iter = std::prev(iter);
      if (prev_iter != navigation_path_list_.end()) {
        const auto &prev_path = std::get<3>(*prev_iter)->path();
        average_point_size = std::min(
            average_point_size,
            std::min(curr_path.path_point_size(), prev_path.path_point_size()));
        double lateral_distance_sum = 0.0;
        for (int i = 0; i < average_point_size; ++i) {
          lateral_distance_sum +=
              fabs(curr_path.path_point(i).y() - prev_path.path_point(i).y());
        }
        double width = lateral_distance_sum /
                       static_cast<double>(average_point_size) / 2.0;
        width = common::math::Clamp(width, config_.min_lane_half_width(),
                                    config_.max_lane_half_width());

        auto &curr_left_width = std::get<1>(*iter);
        auto &prev_right_width = std::get<2>(*prev_iter);
        if (std::get<0>(*iter) == std::get<0>(current_navi_path_tuple_)) {
          prev_right_width = 2.0 * width - curr_left_width;
        } else {
          curr_left_width = width;
          prev_right_width = width;
        }
      }
      // Right neighbor
      auto next_iter = std::next(iter);
      if (next_iter != navigation_path_list_.end()) {
        const auto &next_path = std::get<3>(*next_iter)->path();
        average_point_size = std::min(
            average_point_size,
            std::min(curr_path.path_point_size(), next_path.path_point_size()));
        double lateral_distance_sum = 0.0;
        for (int i = 0; i < average_point_size; ++i) {
          lateral_distance_sum +=
              fabs(curr_path.path_point(i).y() - next_path.path_point(i).y());
        }
        double width = lateral_distance_sum /
                       static_cast<double>(average_point_size) / 2.0;
        width = common::math::Clamp(width, config_.min_lane_half_width(),
                                    config_.max_lane_half_width());

        auto &curr_right_width = std::get<2>(*iter);
        auto &next_left_width = std::get<1>(*next_iter);
        if (std::get<0>(*iter) == std::get<0>(current_navi_path_tuple_)) {
          next_left_width = 2.0 * width - curr_right_width;
        } else {
          next_left_width = width;
          curr_right_width = width;
        }
      }
    }

    return true;
  }

  // Generate a navigation path where the vehicle is located based on perceived
  // lane markers.
  generate_path_on_perception_func();
  return true;
}

double NavigationLane::EvaluateCubicPolynomial(const double c0, const double c1,
                                               const double c2, const double c3,
                                               const double x) const {
  return ((c3 * x + c2) * x + c1) * x + c0;
}

void NavigationLane::MergeNavigationLineAndLaneMarker(
    const int line_index, common::Path *const path) {
  CHECK_NOTNULL(path);

  // If the size of "path" points is smaller than 2, it indicates that a
  // navigation path needs to be generated firstly.
  if (path->path_point_size() < 2) {
    path->Clear();
    ConvertNavigationLineToPath(line_index, path);
  }

  // If the size of "path" points is still smaller than 2, just generate a
  // navigation path based on perceived lane markers.
  if (path->path_point_size() < 2) {
    path->Clear();
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

  int lane_marker_index = 0;
  double navigation_line_weight = 1.0 - config_.lane_marker_weight();
  for (int i = 0; i < path->path_point_size(); ++i) {
    auto *point = path->mutable_path_point(i);
    double s = point->s();
    auto lane_maker_point = GetPathPointByS(lane_marker_path, lane_marker_index,
                                            s, &lane_marker_index);
    // For the beginning and ending portions of a navigation path beyond the
    // perceived path, only the y-coordinates in the FLU coordinate system are
    // used for merging.
    const int marker_size = lane_marker_path.path_point_size();
    if (lane_marker_index < 0 || lane_marker_index > (marker_size - 1)) {
      point->set_y(navigation_line_weight * point->y() +
                   (1 - navigation_line_weight) * lane_maker_point.y());
      lane_marker_index = 0;
      continue;
    }
    *point = common::util::GetWeightedAverageOfTwoPathPoints(
        *point, lane_maker_point, navigation_line_weight,
        1 - navigation_line_weight);
  }
}

common::PathPoint NavigationLane::GetPathPointByS(const common::Path &path,
                                                  const int start_index,
                                                  const double s,
                                                  int *const matched_index) {
  CHECK_NOTNULL(matched_index);
  const int size = path.path_point_size();

  if (start_index < 0 || s < path.path_point(start_index).s()) {
    *matched_index = -1;
    return path.path_point(0);
  }

  if (s > path.path_point(size - 1).s() || start_index > (size - 1)) {
    *matched_index = size;
    return path.path_point(size - 1);
  }

  static constexpr double kEpsilon = 1e-9;
  if (std::fabs(path.path_point(start_index).s() - s) < kEpsilon) {
    *matched_index = start_index;
    return path.path_point(start_index);
  }
  int i = start_index;
  while (i + 1 < size && path.path_point(i + 1).s() < s) {
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

bool NavigationLane::ConvertNavigationLineToPath(const int line_index,
                                                 common::Path *const path) {
  CHECK_NOTNULL(path);
  if (!navigation_info_.navigation_path(line_index).has_path() ||
      navigation_info_.navigation_path(line_index).path().path_point_size() ==
          0) {
    // path is empty
    return false;
  }
  path->set_name(absl::StrCat("Path from navigation line index ", line_index));
  const auto &navigation_path =
      navigation_info_.navigation_path(line_index).path();
  auto proj_index_pair = UpdateProjectionIndex(navigation_path, line_index);
  // Can't find a proper projection index in the "line_index" lane according to
  // current vehicle position.
  int current_project_index = proj_index_pair.first;
  if (current_project_index < 0 ||
      current_project_index >= navigation_path.path_point_size()) {
    AERROR << "Invalid projection index " << current_project_index
           << " in line " << line_index;
    last_project_index_map_.erase(line_index);
    return false;
  } else {
    last_project_index_map_[line_index] = proj_index_pair;
  }

  // offset between the current vehicle state and navigation line
  const double dx = -original_pose_.position().x();
  const double dy = -original_pose_.position().y();
  auto enu_to_flu_func = [this, dx, dy](const double enu_x, const double enu_y,
                                        const double enu_theta, double *flu_x,
                                        double *flu_y, double *flu_theta) {
    if (flu_x != nullptr && flu_y != nullptr) {
      Eigen::Vector2d flu_coordinate = common::math::RotateVector2d(
          {enu_x + dx, enu_y + dy}, -original_pose_.heading());

      *flu_x = flu_coordinate.x();
      *flu_y = flu_coordinate.y();
    }

    if (flu_theta != nullptr) {
      *flu_theta = common::math::NormalizeAngle(
          common::math::NormalizeAngle(enu_theta) - original_pose_.heading());
    }
  };

  auto gen_navi_path_loop_func =
      [this, &navigation_path, &enu_to_flu_func](
          const int start, const int end, const double ref_s_base,
          const double max_length, common::Path *path) {
        CHECK_NOTNULL(path);
        const double ref_s = navigation_path.path_point(start).s();
        for (int i = start; i < end; ++i) {
          auto *point = path->add_path_point();
          point->CopyFrom(navigation_path.path_point(i));

          double flu_x = 0.0;
          double flu_y = 0.0;
          double flu_theta = 0.0;
          enu_to_flu_func(point->x(), point->y(), point->theta(), &flu_x,
                          &flu_y, &flu_theta);

          point->set_x(flu_x);
          point->set_y(flu_y);
          point->set_theta(flu_theta);
          const double accumulated_s =
              navigation_path.path_point(i).s() - ref_s + ref_s_base;
          point->set_s(accumulated_s);

          if (accumulated_s > max_length) {
            break;
          }
        }
      };

  double dist = navigation_path.path_point().rbegin()->s() -
                navigation_path.path_point(current_project_index).s();
  // Stitch current position to the beginning for a cyclic/circular route.
  if (FLAGS_enable_cyclic_rerouting &&
      dist < config_.max_len_from_navigation_line()) {
    auto item_iter = stitch_index_map_.find(line_index);
    if (item_iter != stitch_index_map_.end()) {
      int stitch_start_index =
          std::max(item_iter->second.first, item_iter->second.second);
      stitch_start_index = std::max(current_project_index, stitch_start_index);
      stitch_start_index =
          std::min(navigation_path.path_point_size() - 1, stitch_start_index);

      int stitch_end_index =
          std::min(item_iter->second.first, item_iter->second.second);
      stitch_end_index = std::max(0, stitch_end_index);
      stitch_end_index = std::min(current_project_index, stitch_end_index);

      ADEBUG << "The stitch_start_index is: " << stitch_start_index << "; "
             << "the stitch_end_index is: " << stitch_end_index << "; "
             << "the current_project_index is: " << current_project_index
             << " for the navigation line: " << line_index;

      double length = navigation_path.path_point(stitch_start_index).s() -
                      navigation_path.path_point(current_project_index).s();
      gen_navi_path_loop_func(std::max(0, current_project_index - 3),
                              stitch_start_index + 1, 0.0, length, path);
      if (length > config_.max_len_from_navigation_line()) {
        return true;
      }
      gen_navi_path_loop_func(stitch_end_index,
                              navigation_path.path_point_size(), length,
                              config_.max_len_from_navigation_line(), path);
      return true;
    }
  }

  if (dist < 20) {
    return false;
  }
  gen_navi_path_loop_func(std::max(0, current_project_index - 3),
                          navigation_path.path_point_size(), 0.0,
                          config_.max_len_from_navigation_line(), path);
  return true;
}

// project adc_state_ onto path
ProjIndexPair NavigationLane::UpdateProjectionIndex(const common::Path &path,
                                                    const int line_index) {
  if (path.path_point_size() < 2) {
    return std::make_pair(-1, std::numeric_limits<double>::max());
  }

  double min_d = std::numeric_limits<double>::max();
  const int path_size = path.path_point_size();
  int current_project_index = 0;
  auto item_iter = last_project_index_map_.find(line_index);
  if (item_iter != last_project_index_map_.end()) {
    current_project_index = std::max(0, item_iter->second.first);
  }

  // A lambda expression for checking the distance between the vehicle's initial
  // position and the starting point of  the current navigation line.
  auto check_distance_func = [this, &path, &path_size](
                                 const int project_index,
                                 double *project_distance) {
    // Convert the starting point of the current navigation line from the
    // ENU coordinates to the FLU coordinates. For the multi-lane situation,
    // the distance in the Y-axis direction can be appropriately enlarged,
    // but the distance in the X-axis direction should be small.

    // flu_x = (enu_x - x_shift) * cos(angle) + (enu_y - y_shift) *
    //  sin(angle)
    // flu_y = (enu_y - y_shift) * cos(angle) - (enu_x - x_shift) *
    //  sin(angle)
    if (project_index < 0 || project_index > path_size - 1) {
      return false;
    }
    double enu_x = path.path_point(project_index).x();
    double enu_y = path.path_point(project_index).y();
    double x_shift = original_pose_.position().x();
    double y_shift = original_pose_.position().y();
    double cos_angle = std::cos(original_pose_.heading());
    double sin_angle = std::sin(original_pose_.heading());
    double flu_x =
        (enu_x - x_shift) * cos_angle + (enu_y - y_shift) * sin_angle;
    double flu_y =
        (enu_y - y_shift) * cos_angle - (enu_x - x_shift) * sin_angle;

    if (project_distance != nullptr) {
      *project_distance = std::fabs(flu_y);
    }

    if (std::fabs(flu_x) < config_.max_distance_to_navigation_line() / 2.0 &&
        std::fabs(flu_y) < config_.max_distance_to_navigation_line() * 2.0) {
      return true;
    }
    return false;
  };

  int index = 0;
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

  if (check_distance_func(index, &min_d)) {
    if (FLAGS_enable_cyclic_rerouting) {
      // We create a condition here that sets the "current_project_index" to
      // 0, should the vehicle reach the end point of a cyclic/circular
      // route. For cyclic/circular navigation lines where the distance
      // between their start and end points is very small, it is tedious
      // and unnecessary to re-send navigation lines every time.
      auto item_iter = stitch_index_map_.find(line_index);
      if (item_iter != stitch_index_map_.end()) {
        int start_index =
            std::max(item_iter->second.first, item_iter->second.second);
        int end_index =
            std::min(item_iter->second.first, item_iter->second.second);
        int index_diff = index - start_index;
        if (index_diff >= 0) {
          index = std::min(end_index + index_diff, start_index);
          min_d = DistanceXY(original_pose_.position(), path.path_point(index));
        }
      }
    }
    return std::make_pair(index, min_d);
  }

  return std::make_pair(-1, std::numeric_limits<double>::max());
}

double NavigationLane::GetKappa(const double c1, const double c2,
                                const double c3, const double x) {
  const double dy = 3 * c3 * x * x + 2 * c2 * x + c1;
  const double d2y = 6 * c3 * x + 2 * c2;
  return d2y / std::pow((1 + dy * dy), 1.5);
}

void NavigationLane::ConvertLaneMarkerToPath(
    const perception::LaneMarkers &lane_marker, common::Path *const path) {
  CHECK_NOTNULL(path);

  path->set_name("Path from lane markers.");
  const auto &left_lane = lane_marker.left_lane_marker();
  const auto &right_lane = lane_marker.right_lane_marker();

  double path_c0 = (left_lane.c0_position() + right_lane.c0_position()) / 2.0;

  double path_c1 =
      (left_lane.c1_heading_angle() + right_lane.c1_heading_angle()) / 2.0;

  double path_c2 = (left_lane.c2_curvature() + right_lane.c2_curvature()) / 2.0;

  double path_c3 = (left_lane.c3_curvature_derivative() +
                    right_lane.c3_curvature_derivative()) /
                   2.0;

  const double current_speed =
      vehicle_state_provider_->vehicle_state().linear_velocity();
  double path_range =
      current_speed * config_.ratio_navigation_lane_len_to_speed();
  if (path_range <= config_.min_len_for_navigation_lane()) {
    path_range = config_.min_len_for_navigation_lane();
  } else {
    path_range = config_.max_len_for_navigation_lane();
  }

  const double unit_z = 1.0;
  const double start_s = -2.0;
  double accumulated_s = start_s;
  for (double z = start_s; z <= path_range; z += unit_z) {
    double x1 = z;
    double y1 = 0;
    if (left_lane.view_range() > config_.min_view_range_to_use_lane_marker() ||
        right_lane.view_range() > config_.min_view_range_to_use_lane_marker()) {
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

  perceived_left_width_ = std::fabs(left_lane.c0_position());
  perceived_right_width_ = std::fabs(right_lane.c0_position());
  // If the perceived lane width is incorrect, use the default lane width
  // directly.
  double perceived_lane_width = perceived_left_width_ + perceived_right_width_;
  if (perceived_lane_width < 2.0 * config_.min_lane_half_width() ||
      perceived_lane_width > 2.0 * config_.max_lane_half_width()) {
    perceived_left_width_ = default_left_width_;
    perceived_right_width_ = default_right_width_;
  }
}

bool NavigationLane::CreateMap(const MapGenerationParam &map_config,
                               MapMsg *const map_msg) const {
  auto *navigation_path = map_msg->mutable_navigation_path();
  auto *hdmap = map_msg->mutable_hdmap();
  auto *lane_marker = map_msg->mutable_lane_marker();

  lane_marker->CopyFrom(perception_obstacles_.lane_marker());

  // If no navigation path is generated based on navigation lines, we try to
  // create map with "current_navi_path_tuple_" which is generated based on
  // perceived lane markers.
  if (navigation_path_list_.empty()) {
    if (std::get<3>(current_navi_path_tuple_) != nullptr) {
      FLAGS_relative_map_generate_left_boundray = true;
      return CreateSingleLaneMap(current_navi_path_tuple_, map_config,
                                 perception_obstacles_, hdmap, navigation_path);
    } else {
      return false;
    }
  }

  int fail_num = 0;
  FLAGS_relative_map_generate_left_boundray = true;
  for (auto iter = navigation_path_list_.cbegin();
       iter != navigation_path_list_.cend(); ++iter) {
    std::size_t index = std::distance(navigation_path_list_.cbegin(), iter);
    if (!CreateSingleLaneMap(*iter, map_config, perception_obstacles_, hdmap,
                             navigation_path)) {
      AWARN << "Failed to generate lane: " << index;
      ++fail_num;
      FLAGS_relative_map_generate_left_boundray = true;
      continue;
    }
    FLAGS_relative_map_generate_left_boundray = false;

    // The left border of the middle lane uses the right border of the left
    // lane.
    int lane_index = static_cast<int>(index) - fail_num;
    if (lane_index > 0) {
      auto *left_boundary =
          hdmap->mutable_lane(lane_index)->mutable_left_boundary();
      left_boundary->CopyFrom(hdmap->lane(lane_index - 1).right_boundary());
      auto *left_sample =
          hdmap->mutable_lane(lane_index)->mutable_left_sample();
      left_sample->CopyFrom(hdmap->lane(lane_index - 1).right_sample());
    }
  }

  int lane_num = hdmap->lane_size();
  ADEBUG << "The Lane number is: " << lane_num;

  // Set road boundary
  auto *road = hdmap->add_road();
  road->mutable_id()->set_id("road_" + hdmap->lane(0).id().id());
  auto *section = road->add_section();
  for (int i = 0; i < lane_num; ++i) {
    auto *lane_id = section->add_lane_id();
    lane_id->CopyFrom(hdmap->lane(0).id());
  }
  auto *outer_polygon = section->mutable_boundary()->mutable_outer_polygon();
  auto *left_edge = outer_polygon->add_edge();
  left_edge->set_type(apollo::hdmap::BoundaryEdge::LEFT_BOUNDARY);
  left_edge->mutable_curve()->CopyFrom(hdmap->lane(0).left_boundary().curve());

  auto *right_edge = outer_polygon->add_edge();
  right_edge->set_type(apollo::hdmap::BoundaryEdge::RIGHT_BOUNDARY);
  right_edge->mutable_curve()->CopyFrom(
      hdmap->lane(lane_num - 1).right_boundary().curve());

  // Set neighbor information for each lane
  if (lane_num < 2) {
    return true;
  }
  for (int i = 0; i < lane_num; ++i) {
    auto *lane = hdmap->mutable_lane(i);
    if (i > 0) {
      lane->add_left_neighbor_forward_lane_id()->CopyFrom(
          hdmap->lane(i - 1).id());
      ADEBUG << "Left neighbor is: " << hdmap->lane(i - 1).id().id();
    }
    if (i < lane_num - 1) {
      lane->add_right_neighbor_forward_lane_id()->CopyFrom(
          hdmap->lane(i + 1).id());
      ADEBUG << "Right neighbor is: " << hdmap->lane(i + 1).id().id();
    }
  }
  return true;
}

void NavigationLane::UpdateStitchIndexInfo() {
  stitch_index_map_.clear();

  int navigation_line_num = navigation_info_.navigation_path_size();
  if (navigation_line_num <= 0) {
    return;
  }

  static constexpr int kMinPathPointSize = 10;
  for (int i = 0; i < navigation_line_num; ++i) {
    const auto &navigation_path = navigation_info_.navigation_path(i).path();
    if (!navigation_info_.navigation_path(i).has_path() ||
        navigation_path.path_point_size() < kMinPathPointSize) {
      continue;
    }

    double min_distance = std::numeric_limits<double>::max();
    StitchIndexPair min_index_pair = std::make_pair(-1, -1);

    int path_size = navigation_path.path_point_size();
    const double start_s = navigation_path.path_point(0).s();
    const double end_s = navigation_path.path_point(path_size - 1).s();
    for (int m = 0; m < path_size; ++m) {
      double forward_s = navigation_path.path_point(m).s() - start_s;
      if (forward_s > config_.max_len_from_navigation_line()) {
        break;
      }

      for (int n = path_size - 1; n >= 0; --n) {
        double reverse_s = end_s - navigation_path.path_point(n).s();
        if (reverse_s > config_.max_len_from_navigation_line()) {
          break;
        }
        if (m == n) {
          break;
        }

        double current_distance = DistanceXY(navigation_path.path_point(m),
                                             navigation_path.path_point(n));
        if (current_distance < min_distance) {
          min_distance = current_distance;
          min_index_pair = std::make_pair(m, n);
        }
      }
    }

    if (min_distance < config_.min_lane_half_width()) {
      AINFO << "The stitching pair is: (" << min_index_pair.first << ", "
            << min_index_pair.second << ") for the navigation line: " << i;
      stitch_index_map_[i] = min_index_pair;
    }
  }
}

}  // namespace relative_map
}  // namespace apollo
