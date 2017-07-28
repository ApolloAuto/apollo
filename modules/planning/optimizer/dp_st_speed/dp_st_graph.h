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
 * @file dp_st_graph.h
 **/

#ifndef MODULES_PLANNING_OPTIMIZER_DP_ST_SPEED_DP_ST_GRAPH_H_
#define MODULES_PLANNING_OPTIMIZER_DP_ST_SPEED_DP_ST_GRAPH_H_

#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/dp_st_speed_config.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/decision_data.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/speed/st_point.h"
#include "modules/planning/optimizer/dp_st_speed/dp_st_cost.h"
#include "modules/planning/optimizer/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

class DpStGraph {
 public:
  explicit DpStGraph(const DpStSpeedConfig& dp_config);

  apollo::common::Status Search(const StGraphData& st_graph_data,
                                DecisionData* const decision_data,
                                SpeedData* const speed_data);

 private:
  apollo::common::Status InitCostTable();

  apollo::common::Status CalculatePointwiseCost(
      const std::vector<StGraphBoundary>& boundaries);

  apollo::common::Status CalculateTotalCost();

  apollo::common::Status retrieve_speed_profile(
      SpeedData* const speed_data) const;

  apollo::common::Status get_object_decision(
      const StGraphData& st_graph_data, const SpeedData& speed_profile) const;

  void CalculateTotalCost(const std::uint32_t r, const std::uint32_t c);

  double CalculateEdgeCost(const STPoint& first, const STPoint& second,
                           const STPoint& third, const STPoint& forth,
                           const double speed_limit) const;
  double CalculateEdgeCostForSecondRow(const uint32_t col,
                                       const double speed_limit) const;
  double CalculateEdgeCostForThirdRow(const uint32_t curr_c,
                                      const uint32_t pre_c,
                                      const double speed_limit) const;

  // feasible c_prepre range given c_pre, c
  bool CalculateFeasibleAccelRange(const double c_pre, const double c_cur,
                                   std::uint32_t* const lower_bound,
                                   std::uint32_t* const upper_bound) const;

 private:
  // dp st configuration
  DpStSpeedConfig _dp_st_speed_config;

  // cost utility with configuration;
  DpStCost _dp_st_cost;

  // initial status
  TrajectoryPoint _init_point;

  // mappign obstacle to st graph
  // std::unique_ptr<StBoundaryMapper> _st_mapper = nullptr;

  double _unit_s = 0.0;
  double _unit_t = 0.0;

  std::vector<std::vector<StGraphPoint> > _cost_table;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_DP_ST_SPEED_DP_ST_GRAPH_H_
