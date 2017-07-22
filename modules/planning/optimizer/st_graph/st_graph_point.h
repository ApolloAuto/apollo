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

/*
 *  @file: st_graph_point.h
 */

#ifndef MODULES_PLANNING_OPTIMIZER_ST_GRAPH_ST_GRAPH_POINT_H_
#define MODULES_PLANNING_OPTIMIZER_ST_GRAPH_ST_GRAPH_POINT_H_

#include "modules/planning/common/speed/st_point.h"

namespace apollo {
namespace planning {

class STGraphPoint {
 public:
  ~STGraphPoint() = default;

 public:
  std::size_t index_s() const;
  std::size_t index_t() const;

  const STPoint& point() const;
  const STGraphPoint* pre_point() const;

  double reference_cost() const;
  double obstacle_cost() const;
  double total_cost() const;

  void init(const std::size_t index_t, const std::size_t index_s,
            const STPoint& st_point);

  // given reference speed profile, reach the cost, including position
  void set_reference_cost(const double reference_cost);

  // given obstacle info, get the cost;
  void set_obstacle_cost(const double obs_cost);

  // total cost
  void set_total_cost(const double total_cost);

  void set_pre_point(const STGraphPoint& pre_point);

  STPoint* mutable_point();

 private:
  STPoint _point;
  const STGraphPoint* _pre_point = nullptr;
  std::size_t _index_s = 0;
  std::size_t _index_t = 0;

  double _reference_cost = 0.0;
  double _obstacle_cost = 0.0;
  double _total_cost = 0.0;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_ST_GRAPH_ST_GRAPH_POINT_H_
