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
*   @file: st_graph_boundary.h
**/

#ifndef MODULES_PLANNING_OPTIMIZER_ST_GRAPH_ST_GRAPH_BOUNDARY_H_
#define MODULES_PLANNING_OPTIMIZER_ST_GRAPH_ST_GRAPH_BOUNDARY_H_

#include <memory>

#include "modules/planning/proto/planning.pb.h"

#include "modules/common/math/polygon2d.h"
#include "modules/planning/common/decision.h"
#include "modules/planning/optimizer/st_graph/st_graph_point.h"

namespace apollo {
namespace planning {

class STGraphBoundary final : public ::apollo::common::math::Polygon2d {
 public:
  enum class BoundaryType {
    STOP,
    FOLLOW,
    YIELD,
    OVERTAKE,
    SIDEPASSFOLLOW,
    SIDEPASSLEAD,
    UNKNOWN
  };

  STGraphBoundary(const std::vector<STPoint>& points);
  STGraphBoundary(const std::vector<::apollo::common::math::Vec2d>& points);

  ~STGraphBoundary() = default;

  bool is_empty() const;
  bool IsPointInBoundary(const STGraphPoint& st_graph_point) const;
  bool IsPointInBoundary(const STPoint& st_point) const;

  const ::apollo::common::math::Vec2d point(const size_t index) const;
  const std::vector<::apollo::common::math::Vec2d>& points() const;

  BoundaryType boundary_type() const;
  uint32_t id() const;
  double characteristic_length() const;
  Decision::DecisionType decision_type() const;

  void set_id(const uint32_t id);
  void set_boundary_type(const BoundaryType& boundary_type);
  void set_decision_type(const Decision::DecisionType& decision_type);
  void set_characteristic_length(const double characteristic_length);

  bool get_s_boundary_position(const double curr_time, double* s_upper,
                               double* s_lower) const;

  bool get_boundary_s_range_by_time(const double curr_time, double* s_upper,
                                    double* s_lower) const;

  void get_boundary_time_scope(double* start_t, double* end_t) const;

 private:
  BoundaryType _boundary_type;
  uint32_t _id = 0;
  Decision::DecisionType _decision_type;
  double _characteristic_length = 1.0;
  double _s_high_limit = 200.0;
};

}  // end namespace planning
}  // end namespace apollo

#endif  // BAIDU_ADU_PLANNING_OTIMIZER_COMMON_ST_GRAPH_BOUNDARY_H_
