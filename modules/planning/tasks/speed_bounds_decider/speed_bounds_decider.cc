/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/speed_bounds_decider/speed_bounds_decider.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/path/path_data.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/common/st_graph_data.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/planning/tasks/speed_bounds_decider/speed_limit_decider.h"
#include "modules/planning/tasks/speed_bounds_decider/st_boundary_mapper.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::planning_internal::StGraphBoundaryDebug;
using apollo::planning_internal::STGraphDebug;

bool SpeedBoundsDecider::Init(
    const std::string &config_dir, const std::string &name,
    const std::shared_ptr<DependencyInjector> &injector) {
  if (!Decider::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  return Decider::LoadConfig<SpeedBoundsDeciderConfig>(&config_);
}
/// @brief 
/// @param frame 
/// @param reference_line_info 
/// @return 
Status SpeedBoundsDecider::Process(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  // retrieve data from frame and reference_line_info
  const PathData &path_data = reference_line_info->path_data();
  const TrajectoryPoint &init_point = frame->PlanningStartPoint();
  const ReferenceLine &reference_line = reference_line_info->reference_line();
  PathDecision *const path_decision = reference_line_info->path_decision();

  // 1. Map obstacles into st graph
  // 记录开始时间，创建 STBoundaryMapper 对象用于将障碍物映射到ST图（即时空图）
  auto time1 = std::chrono::system_clock::now();
  STBoundaryMapper boundary_mapper(config_, reference_line, path_data,
                                   path_data.discretized_path().Length(),
                                   config_.total_time(), injector_);

  if (!FLAGS_use_st_drivable_boundary) {   // 默认为false
    path_decision->EraseStBoundaries();
  }
  // 将障碍物投影到ST Graph上
  if (boundary_mapper.ComputeSTBoundary(path_decision).code() ==
      ErrorCode::PLANNING_ERROR) {
    const std::string msg = "Mapping obstacle failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // 记录结束时间，并计算ST边界计算的时间（单位为毫秒），输出调试信息
  auto time2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = time2 - time1;
  ADEBUG << "Time for ST Boundary Mapping = " << diff.count() * 1000
         << " msec.";
  // 所有的障碍物的st_boundary送入到一个boundaries vector之中进行保存
  std::vector<const STBoundary *> boundaries;
  // 遍历路径决策中的所有障碍物，获取每个障碍物的ST边界
  for (auto *obstacle : path_decision->obstacles().Items()) {
    const auto &id = obstacle->Id();
    const auto &st_boundary = obstacle->path_st_boundary();
    // 如果ST边界不为空，判断该边界的类型
    if (!st_boundary.IsEmpty()) {
      // 如果边界类型是 KEEP_CLEAR，表示该障碍物不会阻塞路径，设置为非阻塞
      if (st_boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
        path_decision->Find(id)->SetBlockingObstacle(false);
      } else {
        path_decision->Find(id)->SetBlockingObstacle(true);
      }
      st_boundary.PrintDebug("_obs_st_bounds");
      boundaries.push_back(&st_boundary);
    }
  }
 // 获取速度回退的最小 s 位置（可能是路径上的某个点，用于设置速度限制的回退距离）
  const double min_s_on_st_boundaries = SetSpeedFallbackDistance(path_decision);

  // 2. Create speed limit along path
  SpeedLimitDecider speed_limit_decider(config_, reference_line, path_data);

  SpeedLimit speed_limit;
  if (!speed_limit_decider
           .GetSpeedLimits(path_decision->obstacles(), &speed_limit)
           .ok()) {
    const std::string msg = "Getting speed limits failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 3. Get path_length as s axis search bound in st graph
  // 获取路径的长度（s 轴上的范围）
  const double path_data_length = path_data.discretized_path().Length();

  // 4. Get time duration as t axis search bound in st graph
  // 获取配置中的总时间，用作ST图的 t 轴搜索范围
  const double total_time_by_conf = config_.total_time();

  // Load generated st graph data back to frame
  // 将生成的ST图数据加载到 frame 中
  StGraphData *st_graph_data = reference_line_info_->mutable_st_graph_data();

  // Add a st_graph debug info and save the pointer to st_graph_data for
  // optimizer logging
  // 创建调试信息并保存 st_graph_data 的指针，以便日志记录器使用
  auto *debug = reference_line_info_->mutable_debug();
  STGraphDebug *st_graph_debug = debug->mutable_planning_data()->add_st_graph();

// 将计算的ST图数据加载到 st_graph_data 中，包含边界、最小 s 值、初始点、速度限制、巡航速度等信息
  st_graph_data->LoadData(boundaries, min_s_on_st_boundaries, init_point,
                          speed_limit, reference_line_info->GetCruiseSpeed(),
                          path_data_length, total_time_by_conf, st_graph_debug);

  // Create and record st_graph debug info
  RecordSTGraphDebug(*st_graph_data, st_graph_debug);

  return Status::OK();
}
/// @brief 根据路径决策中存在的障碍物的 st_boundary 来设置速度回退的距离（min_s_on_st_boundaries）
/// @param path_decision 一个指向 PathDecision 类型的常量指针 path_decision，该参数包含了当前路径决策的信息
/// @return 
double SpeedBoundsDecider::SetSpeedFallbackDistance(
    PathDecision *const path_decision) {
  // Set min_s_on_st_boundaries to guide speed fallback.
  static constexpr double kEpsilon = 1.0e-6;
  double min_s_non_reverse = std::numeric_limits<double>::infinity();
  double min_s_reverse = std::numeric_limits<double>::infinity();
// 循环遍历 path_decision 中的每一个障碍物。path_decision->obstacles() 返回一个障碍物列表，Items() 是获取障碍物项的方法，obstacle 是指向每个障碍物的指针
  for (auto *obstacle : path_decision->obstacles().Items()) {
    // 获取当前障碍物的 st_boundary，即障碍物的 s-t 边界
    const auto &st_boundary = obstacle->path_st_boundary();
  // 如果当前障碍物的 st_boundary 是空的（即没有边界数据），则跳过该障碍物，继续下一个循环
    if (st_boundary.IsEmpty()) {
      continue;
    }
   // 分别获取当前障碍物 st_boundary 左下角和右下角的 s 坐标，并计算它们中较小的值
    const auto left_bottom_point_s = st_boundary.bottom_left_point().s();
    const auto right_bottom_point_s = st_boundary.bottom_right_point().s();
    const auto lowest_s = std::min(left_bottom_point_s, right_bottom_point_s);

    if (left_bottom_point_s - right_bottom_point_s > kEpsilon) {
      if (min_s_reverse > lowest_s) {
        min_s_reverse = lowest_s;
      }
    } else if (min_s_non_reverse > lowest_s) {
      min_s_non_reverse = lowest_s;
    }
  }
  // 确保 min_s_reverse 和 min_s_non_reverse 不会小于零。即，最小的 s 值不能为负数
  min_s_reverse = std::max(min_s_reverse, 0.0);
  min_s_non_reverse = std::max(min_s_non_reverse, 0.0);

  return min_s_non_reverse > min_s_reverse ? 0.0 : min_s_non_reverse;
}

void SpeedBoundsDecider::RecordSTGraphDebug(
    const StGraphData &st_graph_data, STGraphDebug *st_graph_debug) const {
  if (!FLAGS_enable_record_debug || !st_graph_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }

  for (const auto &boundary : st_graph_data.st_boundaries()) {
    auto boundary_debug = st_graph_debug->add_boundary();
    boundary_debug->set_name(boundary->id());
    switch (boundary->boundary_type()) {
      case STBoundary::BoundaryType::FOLLOW:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_FOLLOW);
        break;
      case STBoundary::BoundaryType::OVERTAKE:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_OVERTAKE);
        break;
      case STBoundary::BoundaryType::STOP:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_STOP);
        break;
      case STBoundary::BoundaryType::UNKNOWN:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_UNKNOWN);
        break;
      case STBoundary::BoundaryType::YIELD:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_YIELD);
        break;
      case STBoundary::BoundaryType::KEEP_CLEAR:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_KEEP_CLEAR);
        break;
    }

    for (const auto &point : boundary->points()) {
      auto point_debug = boundary_debug->add_point();
      point_debug->set_t(point.x());
      point_debug->set_s(point.y());
    }
  }

  for (const auto &point : st_graph_data.speed_limit().speed_limit_points()) {
    common::SpeedPoint *speed_point = st_graph_debug->add_speed_limit();
    speed_point->set_s(point.first);
    speed_point->set_v(point.second);
  }
}

}  // namespace planning
}  // namespace apollo
