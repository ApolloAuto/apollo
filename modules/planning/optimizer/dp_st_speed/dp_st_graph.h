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
 * @file qp_st_graph.h
 **/

#ifndef BAIDU_IDG_HOUSTON_OPTIMIZER_QP_ST_SPEED_OPTIMIZER_DP_ST_GRAPH_H_
#define BAIDU_IDG_HOUSTON_OPTIMIZER_QP_ST_SPEED_OPTIMIZER_DP_ST_GRAPH_H_

#include "optimizer/st_graph/st_graph.h"

#include "config.pb.h"
#include "planning_halo.pb.h"

#include "optimizer/dp_st_speed_optimizer/dp_st_configuration.h"
#include "optimizer/dp_st_speed_optimizer/dp_st_cost.h"
#include "optimizer/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

class DPSTGraph : public STGraph {
 public:
  DPSTGraph(const DpStConfiguration& dp_config,
            // const STBoundaryConfig& mapper_config,
            const ::adu::common::config::VehicleParam& veh_param);

  ErrorCode search(const STGraphData& st_graph_data,
                   DecisionData* const decision_data,
                   SpeedData* const speed_data);

 private:
  ErrorCode init_cost_table();

  ErrorCode calculate_pointwise_cost(
      const std::vector<STGraphBoundary>& boundaries);

  ErrorCode calculate_total_cost();

  ErrorCode retrieve_speed_profile(SpeedData* const speed_data) const;

  ErrorCode get_object_decision(const STGraphData& st_graph_data,
                                const SpeedData& speed_profile) const;

  void calculate_total_cost(const std::size_t r, const std::size_t c);

  double calculate_edge_cost(const STPoint& first, const STPoint& second,
                             const STPoint& third, const STPoint& forth,
                             const double speed_limit) const;
  double calculate_edge_cost_for_second_row(const size_t col,
                                            const double speed_limit) const;
  double calculate_edge_cost_for_third_row(const size_t curr_c,
                                           const size_t pre_c,
                                           const double speed_limit) const;

  // feasible c_prepre range given c_pre, c
  bool feasible_accel_range(const double c_pre, const double c_cur,
                            std::size_t* const lower_bound,
                            std::size_t* const upper_bound) const;

 private:
  // dp st configuration
  DpStConfiguration _dp_st_configuration;

  // cost utility with configuration;
  DpStCost _dp_st_cost;

  // initial status
  TrajectoryPoint _init_point;

  // mappign obstacle to st graph
  // std::unique_ptr<STBoundaryMapper> _st_mapper = nullptr;

  double _unit_s = 0.0;
  double _unit_t = 0.0;

  std::vector<std::vector<STGraphPoint> > _cost_table;
};

}  // namespace planning
}  // namespace apollo

#endif  // BAIDU_IDG_HOUSTON_OPTIMIZER_QP_ST_SPEED_OPTIMIZER_QP_ST_GRAPH_H_
