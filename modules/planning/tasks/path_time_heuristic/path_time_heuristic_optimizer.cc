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
 * @file path_time_heuristic_optimizer.cc
 **/

#include "modules/planning/tasks/path_time_heuristic/path_time_heuristic_optimizer.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/common/st_graph_data.h"
#include "modules/planning/tasks/path_time_heuristic/gridded_path_time_graph.h"

namespace apollo {
namespace planning {
// https://blog.csdn.net/sinat_52032317/article/details/132612308
using apollo::common::ErrorCode;
using apollo::common::Status;

bool PathTimeHeuristicOptimizer::Init(
    const std::string& config_dir, const std::string& name,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (!SpeedOptimizer::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config_ this task.
  return SpeedOptimizer::LoadConfig<SpeedHeuristicOptimizerConfig>(&config_);
}
/// @brief 
/// @param speed_data 
/// @return 
bool PathTimeHeuristicOptimizer::SearchPathTimeGraph(
    SpeedData* speed_data) const {
  const auto& dp_st_speed_optimizer_config =
      reference_line_info_->IsChangeLanePath()
          ? config_.lane_change_speed_config()
          : config_.default_speed_config();

  GriddedPathTimeGraph st_graph(
      reference_line_info_->st_graph_data(), dp_st_speed_optimizer_config,
      reference_line_info_->path_decision()->obstacles().Items(), init_point_);

  if (!st_graph.Search(speed_data).ok()) {
    AERROR << "failed to search graph with dynamic programming.";
    return false;
  }
  return true;
}
/// @brief 通过动态规划（Dynamic Programming，DP）方法来优化路径的时间分配，并生成对应的速度数据
/// @param path_data 包含路径信息的结构体，通常包括离散化的路径（路径上的多个点）、参考线等
/// @param init_point 初始轨迹点，通常包含车辆的初始位置和速度
/// @param speed_data 存储计算后的速度数据，通常是车辆在路径上每个点的速度信息
/// @return 
Status PathTimeHeuristicOptimizer::Process(
    const PathData& path_data, const common::TrajectoryPoint& init_point,
    SpeedData* const speed_data) {
  init_point_ = init_point;

  if (path_data.discretized_path().empty()) {
    const std::string msg = "Empty path data";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!SearchPathTimeGraph(speed_data)) {
    const std::string msg = absl::StrCat(
        Name(), ": Failed to search graph with dynamic programming.");
    AERROR << msg;
    RecordDebugInfo(*speed_data, reference_line_info_->mutable_st_graph_data()
                                     ->mutable_st_graph_debug());
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  RecordDebugInfo(
      *speed_data,
      reference_line_info_->mutable_st_graph_data()->mutable_st_graph_debug());
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
