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
#include <vector>

#include "modules/common/math/polygon2d.h"
#include "modules/planning/common/speed/st_point.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace planning {

class StBoundary : public common::math::Polygon2d {
 public:
  StBoundary() = default;

  // boundary points go counter clockwise.
  explicit StBoundary(const std::vector<STPoint>& points);

  // boundary points go counter clockwise.
  explicit StBoundary(const std::vector<common::math::Vec2d>& points);
  ~StBoundary() = default;

  // if you need to add boundary type, make sure you modify
  // GetUnblockSRange accordingly.
  enum class BoundaryType {
    UNKNOWN,
    STOP,
    FOLLOW,
    YIELD,
    OVERTAKE,
  };

  bool IsEmpty() const { return points_.empty(); }
  bool IsPointInBoundary(const STPoint& st_point) const;

  STPoint BottomLeftPoint() const;
  STPoint BottomRightPoint() const;
  STPoint TopRightPoint() const;
  STPoint TopLeftPoint() const;

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

 private:
  BoundaryType boundary_type_ = BoundaryType::UNKNOWN;
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
