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

#ifndef MODULES_PLANNING_TASKS_ST_GRAPH_ST_GRAPH_POINT_H_
#define MODULES_PLANNING_TASKS_ST_GRAPH_ST_GRAPH_POINT_H_

#include <limits>

#include "modules/planning/common/speed/st_point.h"

namespace apollo {
namespace planning {

class StGraphPoint {
 public:
  std::uint32_t index_s() const;
  std::uint32_t index_t() const;

  const STPoint& point() const;
  const StGraphPoint* pre_point() const;

  float reference_cost() const;
  float obstacle_cost() const;
  float total_cost() const;

  void Init(const std::uint32_t index_t, const std::uint32_t index_s,
            const STPoint& st_point);

  // given reference speed profile, reach the cost, including position
  void SetReferenceCost(const float reference_cost);

  // given obstacle info, get the cost;
  void SetObstacleCost(const float obs_cost);

  // total cost
  void SetTotalCost(const float total_cost);

  void SetPrePoint(const StGraphPoint& pre_point);

 private:
  STPoint point_;
  const StGraphPoint* pre_point_ = nullptr;
  std::uint32_t index_s_ = 0;
  std::uint32_t index_t_ = 0;

  float reference_cost_ = 0.0;
  float obstacle_cost_ = 0.0;
  float total_cost_ = std::numeric_limits<float>::infinity();
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_ST_GRAPH_ST_GRAPH_POINT_H_
