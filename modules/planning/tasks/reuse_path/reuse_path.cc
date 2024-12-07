/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/reuse_path/reuse_path.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

bool ReusePath::Init(const std::string& config_dir, const std::string& name,
                     const std::shared_ptr<DependencyInjector>& injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  return Task::LoadConfig<ReusePathConfig>(&config_);
}

apollo::common::Status ReusePath::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  if (!reference_line_info->path_data().Empty()) {
    return Status::OK();
  }
  reference_line_info->set_path_reusable(false);
  if (!IsPathReusable(frame, reference_line_info)) {
    path_reusable_ = false;
    return Status::OK();
  }
  path_reusable_ = true;
  if (!TrimHistoryPath(frame, reference_line_info)) {
    path_reusable_ = false;
    return Status::OK();
  }

  reference_line_info->set_path_reusable(true);

  return Status::OK();
}
/// @brief 判断当前路径是否可以被重用，包括车道变更（lane change）、路径规划失败、障碍物碰撞等情况
/// @param frame 当前信息
/// @param reference_line_info 参考路径的信息
/// @return 
bool ReusePath::IsPathReusable(Frame* frame,
                               ReferenceLineInfo* const reference_line_info) {
  // active path reuse during change_lane only
  auto* lane_change_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_change_lane();
  ADEBUG << "lane change status: " << lane_change_status->ShortDebugString();
  // 如果当前路径不是车道变更路径且配置中没有启用车道保持过程中重用路径，则跳过重用路径的判断
  if (!reference_line_info->IsChangeLanePath() &&
      !config_.enable_reuse_path_in_lane_follow()) {
    ADEBUG << "skipping reusing path: not in lane_change";
    return false;
  }
  // 如果当前路径是车道变更路径，但车道变更状态不是“正在变更车道”，则不能重用路径
  if (reference_line_info->IsChangeLanePath() &&
      lane_change_status->status() != ChangeLaneStatus::IN_CHANGE_LANE) {
    return false;
  }
  // stop reusing current path:
  // 1. replan path
  // 2. collision
  // 3. failed to trim previous path
  // 4. speed optimization failed on previous path
  // 停止重用当前路径
  //如果路径重新规划（replan）了；
  //如果发生了碰撞；
  //如果上次路径修剪失败；
  //如果之前的路径速度优化失败
  bool speed_optimization_successful = false;
  // 获取最新的历史帧信息 history_frame
  const auto& history_frame = injector_->frame_history()->Latest();
  // 如果存在历史帧，则判断历史路径的速度优化是否成功（通过轨迹类型是否为 SPEED_FALLBACK）
  if (history_frame) {
    const auto history_trajectory_type =
        history_frame->reference_line_info().front().trajectory_type();
    speed_optimization_successful =
        (history_trajectory_type != ADCTrajectory::SPEED_FALLBACK);
    // 如果历史帧的当前规划路径为空，则返回 false
    if (history_frame->current_frame_planned_path().empty()) {
      return false;
    }
  }
  if (path_reusable_) {
    // 如果当前帧没有重新规划路径（is_replan()），并且速度优化成功且没有碰撞（通过 IsCollisionFree 函数检查），则可以重用路径
    if (!frame->current_frame_planned_trajectory().is_replan() &&
        speed_optimization_successful && IsCollisionFree(reference_line_info)) {
      ADEBUG << "reuse path";
      return true;
    } else {
      // stop reuse path
      ADEBUG << "stop reuse path";
      return false;
    }
  } else {
    // F -> T
    auto* mutable_path_decider_status = injector_->planning_context()
                                            ->mutable_planning_status()
                                            ->mutable_path_decider();
    static constexpr int kWaitCycle = -2;  // wait 2 cycle
   // 获取前方静态障碍物的周期计数器，并判断是否忽略了前方的障碍物（ignore_blocking_obstacle）
    const int front_static_obstacle_cycle_counter =
        mutable_path_decider_status->front_static_obstacle_cycle_counter();
    const bool ignore_blocking_obstacle =
        IsIgnoredBlockingObstacle(reference_line_info);
    ADEBUG << "counter[" << front_static_obstacle_cycle_counter
           << "] IsIgnoredBlockingObstacle[" << ignore_blocking_obstacle << "]";
    // stop reusing current path:
    // 1. blocking obstacle disappeared or moving far away
    // 2. trimming successful
    // 3. no statical obstacle collision.
    // 如果前方的静态障碍物已消失或移动得很远，或者我们可以忽略这个障碍物，并且速度优化成功且没有碰撞，则启用路径重用
    if ((front_static_obstacle_cycle_counter <= kWaitCycle ||
         ignore_blocking_obstacle) &&
        speed_optimization_successful && IsCollisionFree(reference_line_info)) {
      // enable reuse path
      ADEBUG << "reuse path: front_blocking_obstacle ignorable";
      return true;
    }
  }
  return path_reusable_;
}
/// @brief 判断车辆与前方障碍物的距离是否足够远，如果足够远，就可以忽略该障碍物
/// @param reference_line_info 参考线的信息，它包含了参考路径（reference line）和路径上的各种信息
/// @return 
bool ReusePath::IsIgnoredBlockingObstacle(
    ReferenceLineInfo* const reference_line_info) {
  // 获取的是参考线对象
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  // 一个用于判断障碍物是否可以忽略的距离缓冲区，单位是米（30米）
  static constexpr double kSDistBuffer = 30.0;  // meter
  // 一个时间缓冲区，单位是秒（3秒）
  static constexpr int kTimeBuffer = 3;         // second
  // vehicle speed
  // 获取当前车辆的线速度（速度），单位是米每秒（m/s）
  double adc_speed = injector_->vehicle_state()->linear_velocity();
  // 计算最终的缓冲区距离
  double final_s_buffer = std::max(kSDistBuffer, kTimeBuffer * adc_speed);
  // current vehicle s position
  // adc_position_sl 是当前车辆在参考线上的 s 和 l 坐标，s 是沿路径的前进距离，l 是偏移量
  common::SLPoint adc_position_sl;
  // 获取车辆在参考线上的位置
  GetADCSLPoint(reference_line, &adc_position_sl);
  // blocking obstacle start s
  double blocking_obstacle_start_s;  // 前方阻塞障碍物的起始 s 坐标
  // GetBlockingObstacleS 用于获取阻塞障碍物的 s 坐标
  // 如果障碍物的起始 s 值存在，并且障碍物距离车辆的距离大于 final_s_buffer（即车辆和障碍物之间的距离大于缓冲区）
  if (GetBlockingObstacleS(reference_line_info, &blocking_obstacle_start_s) &&
      // distance to blocking obstacle
      (blocking_obstacle_start_s - adc_position_sl.s() > final_s_buffer)) {
  // 如果满足条件，说明当前障碍物距离车辆足够远，可以忽略该障碍物
    ADEBUG << "blocking obstacle distance: "
           << blocking_obstacle_start_s - adc_position_sl.s();
    return true;
  } else {
    return false;
  }
}
/// @brief 获取前方阻塞障碍物的 s 坐标（参考线上的位置），通过查找阻塞障碍物的 ID，获取该障碍物的位置信息，并返回障碍物起始位置的 s 值
/// @param reference_line_info 包含参考路径信息的对象，它提供了参考线和路径决策的相关信息
/// @param blocking_obstacle_s 一个指针，函数通过它返回阻塞障碍物的 s 坐标
/// @return 
bool ReusePath::GetBlockingObstacleS(
    ReferenceLineInfo* const reference_line_info, double* blocking_obstacle_s) {
  // 获取的是路径决策器的状态信息，它存储了与路径选择和规划相关的各种信息
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  // get blocking obstacle ID (front_static_obstacle_id)
  // blocking_obstacle_ID 获取当前阻塞障碍物的 ID。front_static_obstacle_id 是阻塞障碍物的唯一标识符，它用于区分不同的障碍物
  const std::string& blocking_obstacle_ID =
      mutable_path_decider_status->front_static_obstacle_id();
  // indexed_obstacles 是参考线信息中包含的障碍物列表（以 ID 为索引）。path_decision()->obstacles() 获取当前路径决策中的所有障碍物
  const IndexedList<std::string, Obstacle>& indexed_obstacles =
      reference_line_info->path_decision()->obstacles();
  // Find(blocking_obstacle_ID) 方法通过 ID 查找并返回对应的阻塞障碍物。如果找不到障碍物，则返回 nullptr
  const auto* blocking_obstacle = indexed_obstacles.Find(blocking_obstacle_ID);
  // 如果 blocking_obstacle 为 nullptr，表示未找到阻塞障碍物
  if (blocking_obstacle == nullptr) {
    return false;
  }
  // PerceptionSLBoundary() 获取障碍物的 SL 边界（即障碍物在参考线上的起始和终止位置）
  const auto& obstacle_sl = blocking_obstacle->PerceptionSLBoundary();
  // obstacle_sl.start_s() 获取该障碍物的起始 s 坐标，并将其赋值给传入的 blocking_obstacle_s 指针
  *blocking_obstacle_s = obstacle_sl.start_s();
  ADEBUG << "blocking obstacle distance: " << obstacle_sl.start_s();
  return true;
}
/// @brief 获取车辆的当前位置，并将该位置从笛卡尔坐标系转换为参考路径上的 S-L 坐标系，最后将结果存储在 adc_position_sl 中
/// @param reference_line 
/// @param adc_position_sl 
void ReusePath::GetADCSLPoint(const ReferenceLine& reference_line,
                              common::SLPoint* adc_position_sl) {
 // 车辆在笛卡尔坐标系中的位置
  common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                      injector_->vehicle_state()->y()};
  reference_line.XYToSL(adc_position, adc_position_sl);
}
/// @brief 
/// @param reference_line_info 
/// @return 
bool ReusePath::IsCollisionFree(ReferenceLineInfo* const reference_line_info) {
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  static constexpr double kMinObstacleArea = 1e-4;
  const double kSBuffer = 0.5;
  // current vehicle sl position
  common::SLPoint adc_position_sl;
  // 设置参数，获取自车的SL坐标
  GetADCSLPoint(reference_line, &adc_position_sl);

  // current obstacles
  std::vector<Polygon2d> obstacle_polygons;
  // 遍历当前的障碍物，忽略掉所有动态、虚拟的障碍物
  for (auto obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    // filtered all non-static objects and virtual obstacle
    if (!obstacle->IsStatic() || obstacle->IsVirtual()) {
      if (!obstacle->IsStatic()) {
        ADEBUG << "SPOT a dynamic obstacle";
      }
      if (obstacle->IsVirtual()) {
        ADEBUG << "SPOT a virtual obstacle";
      }
      continue;
    }
    // 忽略掉自车后面的障碍物以及过小的障碍物
    const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
    // Ignore obstacles behind ADC
    if ((obstacle_sl.end_s() < adc_position_sl.s() - kSBuffer) ||
        // Ignore too small obstacles.
        (obstacle_sl.end_s() - obstacle_sl.start_s()) *
                (obstacle_sl.end_l() - obstacle_sl.start_l()) <
            kMinObstacleArea) {
      continue;
    }
    obstacle_polygons.push_back(
        Polygon2d({Vec2d(obstacle_sl.start_s(), obstacle_sl.start_l()),
                   Vec2d(obstacle_sl.start_s(), obstacle_sl.end_l()),
                   Vec2d(obstacle_sl.end_s(), obstacle_sl.end_l()),
                   Vec2d(obstacle_sl.end_s(), obstacle_sl.start_l())}));
  }

  if (obstacle_polygons.empty()) {
    return true;
  }

  const auto& history_frame = injector_->frame_history()->Latest();
  if (!history_frame) {
    return false;
  }
  const DiscretizedPath& history_path =
      history_frame->current_frame_planned_path();
  if (history_path.empty()) {
    AINFO << "No history path skip reuse";
    return false;
  }
  // path end point
  // 将上一段轨迹的终点投影到SL坐标系下
  common::SLPoint path_end_position_sl;
  common::math::Vec2d path_end_position = {history_path.back().x(),
                                           history_path.back().y()};
  reference_line.XYToSL(path_end_position, &path_end_position_sl);
  const double min_distance_to_end =
      FLAGS_path_bounds_decider_resolution * FLAGS_num_extra_tail_bound_point;
  for (size_t i = 0; i < history_path.size(); ++i) {
    common::SLPoint path_position_sl;
    common::math::Vec2d path_position = {history_path[i].x(),
                                         history_path[i].y()};
    reference_line.XYToSL(path_position, &path_position_sl);
    if (path_end_position_sl.s() - path_position_sl.s() <=
        min_distance_to_end) {
      break;
    }
    if (path_position_sl.s() < adc_position_sl.s() - kSBuffer) {
      continue;
    }
    const auto& vehicle_box =
        common::VehicleConfigHelper::Instance()->GetBoundingBox(
            history_path[i]);
    std::vector<Vec2d> ABCDpoints = vehicle_box.GetAllCorners();
    for (const auto& corner_point : ABCDpoints) {
      // For each corner point, project it onto reference_line
      common::SLPoint curr_point_sl;
      if (!reference_line.XYToSL(corner_point, &curr_point_sl)) {
        AERROR << "Failed to get the projection from point onto "
                  "reference_line";
        return false;
      }
      auto curr_point = Vec2d(curr_point_sl.s(), curr_point_sl.l());
      // Check if it's in any polygon of other static obstacles.
      for (const auto& obstacle_polygon : obstacle_polygons) {
        if (obstacle_polygon.IsPointIn(curr_point)) {
          // for debug
          ADEBUG << "s distance to end point:" << path_end_position_sl.s();
          ADEBUG << "s distance to end point:" << path_position_sl.s();
          ADEBUG << "[" << i << "]"
                 << ", history_path[i].x(): " << std::setprecision(9)
                 << history_path[i].x() << ", history_path[i].y()"
                 << std::setprecision(9) << history_path[i].y();
          ADEBUG << "collision:" << curr_point.x() << ", " << curr_point.y();
          Vec2d xy_point;
          reference_line.SLToXY(curr_point_sl, &xy_point);
          ADEBUG << "collision:" << xy_point.x() << ", " << xy_point.y();

          return false;
        }
      }
    }
  }
  return true;
}

// check the length of the path
bool ReusePath::NotShortPath(const DiscretizedPath& current_path) {
  // TODO(shu): use gflag
  return current_path.size() >= config_.short_path_threshold();
}
/// @brief 
/// @param frame 当前的帧数据（类型为 Frame*），可能包含当前车辆状态、路径规划点等信息
/// @param reference_line_info 包含参考线（reference_line）和相关路径数据（path_data）的信息
/// @return 
bool ReusePath::TrimHistoryPath(Frame* frame,
                                ReferenceLineInfo* const reference_line_info) {
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  const auto& history_frame = injector_->frame_history()->Latest();
  if (!history_frame) {
    ADEBUG << "no history frame";
    return false;
  }
  // 从历史帧中获取规划起点（PlanningStartPoint）并获取对应的路径点（PathPoint）
  const common::TrajectoryPoint history_planning_start_point =
      history_frame->PlanningStartPoint();
  common::PathPoint history_init_path_point =
      history_planning_start_point.path_point();
  ADEBUG << "history_init_path_point x:[" << std::setprecision(9)
         << history_init_path_point.x() << "], y["
         << history_init_path_point.y() << "], s: ["
         << history_init_path_point.s() << "]";
  // 获取当前帧的规划起点，并提取路径点信息，输出当前路径点的坐标和弧长信息用于调试
  const common::TrajectoryPoint planning_start_point =
      frame->PlanningStartPoint();
  common::PathPoint init_path_point = planning_start_point.path_point();
  ADEBUG << "init_path_point x:[" << std::setprecision(9) << init_path_point.x()
         << "], y[" << init_path_point.y() << "], s: [" << init_path_point.s()
         << "]";
  // 获取历史帧的规划路径（history_path）
  const DiscretizedPath& history_path =
      history_frame->current_frame_planned_path();
  // 存储修剪后的路径
  DiscretizedPath trimmed_path;
  common::SLPoint adc_position_sl;  // current vehicle sl position
  // 将车辆当前位置转换为 SL 坐标系（弧长 s 和横向位置 l）
  GetADCSLPoint(reference_line, &adc_position_sl);
  ADEBUG << "adc_position_sl.s(): " << adc_position_sl.s();
  ADEBUG << "history_path.size(): " << history_path.size();
  // 遍历历史路径，找到第一个弧长 s 大于 0 的点，设定为路径的起始点。输出该起始点的索引
  size_t path_start_index = 0;

  for (size_t i = 0; i < history_path.size(); ++i) {
    // find previous init point
    if (history_path[i].s() > 0) {
      // // 找到上周期轨迹规划的起点索引
      path_start_index = i;
      break;
    }
  }
  ADEBUG << "!!!path_start_index[" << path_start_index << "]";

  // get current s=0
  // 将当前路径点转换为 SL 坐标系，并初始化一个布尔变量 inserted_init_point，用于标记是否已经插入了当前的起始点
  common::SLPoint init_path_position_sl;
  reference_line.XYToSL(init_path_point, &init_path_position_sl);
  bool inserted_init_point = false;

// 从 path_start_index 开始，遍历历史路径中的每一个点，将其从笛卡尔坐标（x, y）转换为 SL 坐标。
  for (size_t i = path_start_index; i < history_path.size(); ++i) {
    common::SLPoint path_position_sl;
    common::math::Vec2d path_position = {history_path[i].x(),
                                         history_path[i].y()};

    reference_line.XYToSL(path_position, &path_position_sl);
   // 计算上一帧中每个路径点与当前起始点之间的弧长差 updated_s
    double updated_s = path_position_sl.s() - init_path_position_sl.s();
    // insert init point
    // 如果 updated_s > 0 且当前还没有插入起始点，就将当前路径点 init_path_point 插入到 trimmed_path 中，并设置其弧长为 0
    if (updated_s > 0 && !inserted_init_point) {
      trimmed_path.emplace_back(init_path_point);
      trimmed_path.back().set_s(0);
      inserted_init_point = true;
    }
    // 然后将历史路径中的每个点插入到 trimmed_path 
    trimmed_path.emplace_back(history_path[i]);

    // if (i < 50) {
    //   ADEBUG << "path_point:[" << i << "]" << updated_s;
    //   path_position_sl.s();
    //   ADEBUG << std::setprecision(9) << "path_point:[" << i << "]"
    //          << "x: [" << history_path[i].x() << "], y:[" <<
    //          history_path[i].y()
    //          << "]. s[" << history_path[i].s() << "]";
    // }
    // 更新每个点的弧长值 s
    trimmed_path.back().set_s(updated_s);
  }

  ADEBUG << "trimmed_path[0]: " << trimmed_path.front().s();
  ADEBUG << "[END] trimmed_path.size(): " << trimmed_path.size();

  if (!NotShortPath(trimmed_path)) {
    ADEBUG << "short path: " << trimmed_path.size();
    return false;
  }

  // set path
  auto path_data = reference_line_info->mutable_path_data();
  ADEBUG << "previous path_data size: " << history_path.size();
  path_data->SetReferenceLine(&reference_line);
  ADEBUG << "previous path_data size: " << path_data->discretized_path().size();
  path_data->SetDiscretizedPath(DiscretizedPath(std::move(trimmed_path)));
  ADEBUG << "not short path: " << trimmed_path.size();
  ADEBUG << "current path size: "
         << reference_line_info->path_data().discretized_path().size();
  RecordDebugInfo(*path_data, path_data->path_label(), reference_line_info);
  return true;
}

}  // namespace planning
}  // namespace apollo
