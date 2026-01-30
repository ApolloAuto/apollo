/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/common_msgs/planning_msgs/sl_boundary.pb.h"

#include "cyber/common/log.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"

namespace apollo {
namespace planning {

using apollo::common::SLPoint;
using apollo::perception::PerceptionObstacle;

class SLPolygon {
 public:
  enum NudgeType {
    LEFT_NUDGE,
    RIGHT_NUDGE,
    UNDEFINED,
    BLOCKED,
    IGNORE,
  };
  SLPolygon() = default;
  explicit SLPolygon(
      SLBoundary sl_boundary, std::string id = "",
      PerceptionObstacle::Type type = PerceptionObstacle::UNKNOWN,
      bool print_log = false);
  virtual ~SLPolygon() = default;
  static double GetInterpolatedLFromBoundary(
      const std::vector<SLPoint>& boundary, const double s);
  const std::vector<SLPoint>& LeftBoundary() const { return left_boundary_; }
  const std::vector<SLPoint>& RightBoundary() const { return right_boundary_; }
  const double GetLeftBoundaryByS(const double s) const {
    if (nudge_type_ == NudgeType::RIGHT_NUDGE) {
      return std::numeric_limits<double>::max();
    }
    return GetInterpolatedSFromBoundary(left_boundary_, s);
  }
  const double GetRightBoundaryByS(const double s) const {
    if (nudge_type_ == NudgeType::LEFT_NUDGE) {
      return std::numeric_limits<double>::lowest();
    }
    return GetInterpolatedSFromBoundary(right_boundary_, s);
  }
  static double GetInterpolatedSFromBoundary(
      const std::vector<SLPoint>& boundary, double s);
  double MinS() const { return min_s_point_.s(); }
  double MaxS() const { return max_s_point_.s(); }
  double MinL() const { return min_l_point_.l(); }
  double MaxL() const { return max_l_point_.l(); }
  SLPoint MinSPoint() const { return min_s_point_; }
  SLPoint MaxSPoint() const { return max_s_point_; }
  SLPoint MinLPoint() const { return min_l_point_; }
  SLPoint MaxLPoint() const { return max_l_point_; }
  NudgeType NudgeInfo() const { return nudge_type_; }
  void SetNudgeInfo(NudgeType nudge_type) { nudge_type_ = nudge_type; }
  void PrintToLog(std::string prefix = "") const;
  void PrintToLogBlock() const;
  std::string id() const { return id_; }

  void SetOverlapeWithReferCenter(bool overlap) {
    overlape_with_refer_center_ = overlap;
  }
  bool OverlapeWithReferCenter() const { return overlape_with_refer_center_; }
  void SetOverlapeSizeWithReference(double abs_l) {
    overlape_size_with_reference_ =
        std::max(abs_l, overlape_size_with_reference_);
  }
  double OverlapeSizeWithReference() const {
    return overlape_size_with_reference_;
  }

  std::array<bool, 2>& is_passable() { return is_passable_; }
  void UpdatePassableInfo(double left_bound, double right_bound,
                          double left_buffer, double right_buffer,
                          double check_s);
  double MinRadiusStopDistance(double adc_min_l, double adc_max_l,
                               double ego_half_width);
  PerceptionObstacle::Type ObstacleType() const { return obstacle_type_; }

 private:
  std::vector<SLPoint> left_boundary_;
  std::vector<SLPoint> right_boundary_;
  SLBoundary sl_boundary_;
  SLPoint min_s_point_;
  SLPoint max_s_point_;
  SLPoint min_l_point_;
  SLPoint max_l_point_;
  std::string id_;
  bool overlape_with_refer_center_ = false;
  double overlape_size_with_reference_ = 0.0;
  NudgeType nudge_type_ = NudgeType::UNDEFINED;
  std::array<bool, 2> is_passable_ = {true, true};
  double min_radius_stop_distance_ = -1.0;
  PerceptionObstacle::Type obstacle_type_ = PerceptionObstacle::UNKNOWN;
};

}  // namespace planning
}  // namespace apollo
