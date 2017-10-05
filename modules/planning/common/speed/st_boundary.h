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

#ifndef MODULES_PLANNING_TASKS_ST_GRAPH_ST_GRAPH_BOUNDARY_H_
#define MODULES_PLANNING_TASKS_ST_GRAPH_ST_GRAPH_BOUNDARY_H_

#include <limits>
#include <memory>
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

class StBoundary : public common::math::Polygon2d {
 public:
  StBoundary() = default;

  explicit StBoundary(
      const std::vector<std::pair<STPoint, STPoint>>& point_pairs);

  explicit StBoundary(const common::math::Box2d& box) = delete;
  explicit StBoundary(std::vector<common::math::Vec2d> points) = delete;

  ~StBoundary() = default;

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

  bool IsEmpty() const { return lower_points_.empty(); }
  bool IsPointInBoundary(const STPoint& st_point) const;

  STPoint BottomLeftPoint() const;
  STPoint BottomRightPoint() const;

  StBoundary ExpandByS(const double s) const;
  StBoundary ExpandByT(const double t) const;

  BoundaryType boundary_type() const;
  const std::string& id() const;
  double characteristic_length() const;

  void SetId(const std::string& id);
  void SetBoundaryType(const BoundaryType& boundary_type);
  void SetCharacteristicLength(const double characteristic_length);

  bool GetUnblockSRange(const double curr_time, double* s_upper,
                        double* s_lower) const;

  bool GetBoundarySRange(const double curr_time, double* s_upper,
                         double* s_lower) const;

  double min_s() const;
  double min_t() const;
  double max_s() const;
  double max_t() const;

  double Area() const;

  double DistanceS(const STPoint& st_point) const;
  std::vector<STPoint> upper_points() const { return upper_points_; }
  std::vector<STPoint> lower_points() const { return lower_points_; }

  static StBoundary GenerateStBoundary(
      const std::vector<STPoint>& lower_points,
      const std::vector<STPoint>& upper_points);

 private:
  bool IsValid(
      const std::vector<std::pair<STPoint, STPoint>>& point_pairs) const;

  bool IsPointNear(const common::math::LineSegment2d& seg,
                   const common::math::Vec2d& point, const double max_dist);

  FRIEND_TEST(StBoundaryTest, remove_redundant_points);
  void RemoveRedundantPoints(
      std::vector<std::pair<STPoint, STPoint>>* point_pairs);
  void CalculateArea();

  FRIEND_TEST(StBoundaryTest, get_index_range);
  bool GetIndexRange(const std::vector<STPoint>& points, const double t,
                     size_t* left, size_t* right) const;

 private:
  BoundaryType boundary_type_ = BoundaryType::UNKNOWN;

  std::vector<STPoint> upper_points_;
  std::vector<STPoint> lower_points_;

  double area_ = 0.0;

  std::string id_;
  double characteristic_length_ = 1.0;
  double s_high_limit_ = 200.0;
  double min_s_ = std::numeric_limits<double>::max();
  double max_s_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  double max_t_ = std::numeric_limits<double>::lowest();
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_ST_GRAPH_ST_GRAPH_BOUNDARY_H_
