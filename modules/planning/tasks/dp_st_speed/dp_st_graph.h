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

#ifndef MODULES_PLANNING_TASKS_DP_ST_SPEED_DP_ST_GRAPH_H_
#define MODULES_PLANNING_TASKS_DP_ST_SPEED_DP_ST_GRAPH_H_

#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/dp_st_speed_config.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/speed/st_point.h"
#include "modules/planning/tasks/dp_st_speed/dp_st_cost.h"
#include "modules/planning/tasks/dp_st_speed/st_graph_point.h"
#include "modules/planning/tasks/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

class DpStGraph {
 public:
  DpStGraph(const ReferenceLine& reference_line,
            const StGraphData& st_graph_data, const DpStSpeedConfig& dp_config,
            const PathData& path_data, const SLBoundary& adc_sl_boundary);

  apollo::common::Status Search(PathDecision* const path_decision,
                                SpeedData* const speed_data);

 private:
  apollo::common::Status InitCostTable();

  void CalculatePointwiseCost(const std::vector<const StBoundary*>& boundaries);

  apollo::common::Status RetrieveSpeedProfile(
      SpeedData* const speed_data) const;

  apollo::common::Status CalculateTotalCost();
  void CalculateCostAt(const uint32_t r, const uint32_t c);

  double CalculateEdgeCost(const STPoint& first, const STPoint& second,
                           const STPoint& third, const STPoint& forth,
                           const double speed_limit) const;
  double CalculateEdgeCostForSecondCol(const uint32_t row,
                                       const double speed_limit) const;
  double CalculateEdgeCostForThirdCol(const uint32_t curr_r,
                                      const uint32_t pre_r,
                                      const double speed_limit) const;

  bool CalculateFeasibleAccelRange(const double r_pre, const double r_cur,
                                   uint32_t* const lower_bound,
                                   uint32_t* const upper_bound) const;

  void GetRowRange(const StGraphPoint& point, uint32_t* highest_row,
                   uint32_t* lowest_row);

 private:
  const ReferenceLine& reference_line_;
  // dp st configuration
  DpStSpeedConfig dp_st_speed_config_;

  const StGraphData& st_graph_data_;

  // vehicle configuration parameter
  const common::VehicleParam& vehicle_param_;

  const SLBoundary& adc_sl_boundary_;

  // cost utility with configuration;
  DpStCost dp_st_cost_;

  // initial status
  common::TrajectoryPoint init_point_;

  double unit_s_ = 0.0;
  double unit_t_ = 0.0;

  // cost_table_[t][s]
  // row: s, col: t --- NOTICE: Please do NOT change.
  std::vector<std::vector<StGraphPoint>> cost_table_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_DP_ST_SPEED_DP_ST_GRAPH_H_
