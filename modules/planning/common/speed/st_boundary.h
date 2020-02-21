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
 *   @file
 **/

#pragma once

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest_prod.h"

#include "modules/planning/proto/planning.pb.h"

#include "modules/common/math/box2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/speed/st_point.h"

namespace apollo {
namespace planning {

class STBoundary : public common::math::Polygon2d {
 public:
  /** Constructors:
   *   STBoundary must be initialized with a vector of ST-point pairs.
   *   Each pair refers to a time t, with (lower_s, upper_s).
   */
  STBoundary() = default;
  explicit STBoundary(
      const std::vector<std::pair<STPoint, STPoint>>& point_pairs,
      bool is_accurate_boundary = false);
  explicit STBoundary(const common::math::Box2d& box) = delete;
  explicit STBoundary(std::vector<common::math::Vec2d> points) = delete;

  /** @brief Wrapper of the constructor (old).
   */
  static STBoundary CreateInstance(const std::vector<STPoint>& lower_points,
                                   const std::vector<STPoint>& upper_points);

  /** @brief Wrapper of the constructor. It doesn't use RemoveRedundantPoints
   * and generates an accurate ST-boundary.
   */
  static STBoundary CreateInstanceAccurate(
      const std::vector<STPoint>& lower_points,
      const std::vector<STPoint>& upper_points);

  /** @brief Default destructor.
   */
  ~STBoundary() = default;

  bool IsEmpty() const { return lower_points_.empty(); }

  bool GetUnblockSRange(const double curr_time, double* s_upper,
                        double* s_lower) const;

  bool GetBoundarySRange(const double curr_time, double* s_upper,
                         double* s_lower) const;

  bool GetBoundarySlopes(const double curr_time, double* ds_upper,
                         double* ds_lower) const;

  // if you need to add boundary type, make sure you modify
  // GetUnblockSRange accordingly.
  enum class BoundaryType {
    UNKNOWN,
    STOP,
    FOLLOW,
    YIELD,
    OVERTAKE,
    KEEP_CLEAR,
  };

  static std::string TypeName(BoundaryType type);

  BoundaryType boundary_type() const;
  const std::string& id() const;
  double characteristic_length() const;

  void set_id(const std::string& id);
  void SetBoundaryType(const BoundaryType& boundary_type);
  void SetCharacteristicLength(const double characteristic_length);

  double min_s() const;
  double min_t() const;
  double max_s() const;
  double max_t() const;

  std::vector<STPoint> upper_points() const { return upper_points_; }
  std::vector<STPoint> lower_points() const { return lower_points_; }

  // Used by st-optimizer.
  bool IsPointInBoundary(const STPoint& st_point) const;
  STBoundary ExpandByS(const double s) const;
  STBoundary ExpandByT(const double t) const;

  // Unused function so far.
  STBoundary CutOffByT(const double t) const;

  // Used by Lattice planners.
  STPoint upper_left_point() const;
  STPoint upper_right_point() const;
  STPoint bottom_left_point() const;
  STPoint bottom_right_point() const;

  void set_upper_left_point(STPoint st_point);
  void set_upper_right_point(STPoint st_point);
  void set_bottom_left_point(STPoint st_point);
  void set_bottom_right_point(STPoint st_point);

  void set_obstacle_road_right_ending_t(double road_right_ending_t) {
    obstacle_road_right_ending_t_ = road_right_ending_t;
  }
  double obstacle_road_right_ending_t() const {
    return obstacle_road_right_ending_t_;
  }

 private:
  /** @brief The sanity check function for a vector of ST-point pairs.
   */
  bool IsValid(
      const std::vector<std::pair<STPoint, STPoint>>& point_pairs) const;

  /** @brief Returns true if point is within max_dist distance to seg.
   */
  bool IsPointNear(const common::math::LineSegment2d& seg,
                   const common::math::Vec2d& point, const double max_dist);

  /** @brief Sometimes a sequence of upper and lower points lie almost on
   * two straightlines. In this case, the intermediate points are removed,
   * with only the end-points retained.
   */
  // TODO(all): When slope is high, this may introduce significant errors.
  // Also, when accumulated for multiple t, the error can get significant.
  // This function should be reconsidered, because it may be dangerous.
  void RemoveRedundantPoints(
      std::vector<std::pair<STPoint, STPoint>>* point_pairs);
  FRIEND_TEST(StBoundaryTest, remove_redundant_points);

  /** @brief Given time t, find a segment denoted by left and right idx, that
   * contains the time t.
   * - If t is less than all or larger than all, return false.
   */
  bool GetIndexRange(const std::vector<STPoint>& points, const double t,
                     size_t* left, size_t* right) const;
  FRIEND_TEST(StBoundaryTest, get_index_range);

 private:
  BoundaryType boundary_type_ = BoundaryType::UNKNOWN;

  std::vector<STPoint> upper_points_;
  std::vector<STPoint> lower_points_;

  std::string id_;
  double characteristic_length_ = 1.0;
  double min_s_ = std::numeric_limits<double>::max();
  double max_s_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  double max_t_ = std::numeric_limits<double>::lowest();

  STPoint bottom_left_point_;
  STPoint bottom_right_point_;
  STPoint upper_left_point_;
  STPoint upper_right_point_;

  double obstacle_road_right_ending_t_;
};

}  // namespace planning
}  // namespace apollo
