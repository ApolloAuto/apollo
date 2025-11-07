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
 * @file
 **/

#include "modules/planning/tasks/path_decider/path_decider.h"

#include <algorithm>
#include <memory>
#include "modules/common_msgs/planning_msgs/decision.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/util.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

bool PathDecider::Init(const std::string &config_dir, const std::string &name,
                       const std::shared_ptr<DependencyInjector> &injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  return Task::LoadConfig<PathDeciderConfig>(&config_);
}

Status PathDecider::Execute(Frame *frame,
                            ReferenceLineInfo *reference_line_info) {
  Task::Execute(frame, reference_line_info);
  return Process(reference_line_info, reference_line_info->path_data(),
                 reference_line_info->path_decision());
}

/// @brief 根据生成好的路径 path_data 和当前障碍物信息，更新规划状态、识别阻塞障碍物、并为障碍物决策 path_decision 做准备
/// @param reference_line_info 
/// @param path_data 
/// @param path_decision 
/// @return 
Status PathDecider::Process(const ReferenceLineInfo *reference_line_info,
                            const PathData &path_data,
                            PathDecision *const path_decision) {
  // 启用跳过路径任务、同时路径是可复用的
  // skip path_decider if reused path
  if (FLAGS_enable_skip_path_tasks && reference_line_info->path_reusable()) {
    return Status::OK();
  }

  // 创建一个用于调试绘图的对象 debug_info
  PrintCurves debug_info;
  // 从 path_data 中取出离散化路径
  const auto &path = path_data.discretized_path();

  // 获取路径起点对应的车辆包围盒，并将其所有角点添加到调试输出中
  if (!path.empty()) {
    const auto &vehicle_box =
        common::VehicleConfigHelper::Instance()->GetBoundingBox(path[0]);
    debug_info.AddPoint("start_point_box", vehicle_box.GetAllCorners());
  }

  // 把路径中所有点的坐标加入调试曲线
  for (const auto &path_pt : path) {
    debug_info.AddPoint("output_path", path_pt.x(), path_pt.y());
  }
  // 输出调试信息到日志中
  debug_info.PrintToLog();

  // 初始化阻塞障碍物 ID（blocking_obstacle_id）
  std::string blocking_obstacle_id;
  // 获取 planning_context 中的 path_decider_status 对象，用于记录阻塞障碍物状态
  auto *mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  // 如果当前参考线上有阻塞障碍物
  if (reference_line_info->GetBlockingObstacle() != nullptr) {
    // 获取该障碍物 ID 作为阻塞障碍物
    blocking_obstacle_id = reference_line_info->GetBlockingObstacle()->Id();
    // 保证周期计数器大于等于 0（代表出现过）
    int front_static_obstacle_cycle_counter =
        mutable_path_decider_status->front_static_obstacle_cycle_counter();
    mutable_path_decider_status->set_front_static_obstacle_cycle_counter(
        std::max(front_static_obstacle_cycle_counter, 0));
    // 每次检测到阻塞障碍物，将计数器上限增加到 10（最多累计 10 次）
    mutable_path_decider_status->set_front_static_obstacle_cycle_counter(
        std::min(front_static_obstacle_cycle_counter + 1, 10));
    // 在 planning_context 中记录阻塞障碍物的 ID
    mutable_path_decider_status->set_front_static_obstacle_id(
        reference_line_info->GetBlockingObstacle()->Id());
  } else {
    // 如果当前没有检测到阻塞障碍物
    // 确保计数器不为正数
    int front_static_obstacle_cycle_counter =
        mutable_path_decider_status->front_static_obstacle_cycle_counter();
    mutable_path_decider_status->set_front_static_obstacle_cycle_counter(
        std::min(front_static_obstacle_cycle_counter, 0));
    // 每次未检测到阻塞物，计数器减少一次，最低为 -10
    mutable_path_decider_status->set_front_static_obstacle_cycle_counter(
        std::max(front_static_obstacle_cycle_counter - 1, -10));
    // 如果连续多帧（超过2次）未检测到障碍物，将 front_static_obstacle_id 清空（设为空格字符串）
    if (mutable_path_decider_status->front_static_obstacle_cycle_counter() <
        -2) {
      std::string id = " ";
      mutable_path_decider_status->set_front_static_obstacle_id(id);
    }
  }
  if (!MakeObjectDecision(path_data, blocking_obstacle_id, path_decision)) {
    const std::string msg = "Failed to make decision based on tunnel";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

/// @brief 生成针对路径上的障碍物的决策（如绕行、停车、忽略等）
/// @param path_data 当前规划路径数据
/// @param blocking_obstacle_id 路径上认为“阻塞”的主要障碍物 ID
/// @param path_decision 最终决策会写入这个对象中
/// @return 
bool PathDecider::MakeObjectDecision(const PathData &path_data,
                                     const std::string &blocking_obstacle_id,
                                     PathDecision *const path_decision) {
  if (!MakeStaticObstacleDecision(path_data, blocking_obstacle_id,
                                  path_decision)) {
    AERROR << "Failed to make decisions for static obstacles";
    return false;
  }
  if (config_.ignore_backward_obstacle()) {
    IgnoreBackwardObstacle(path_decision);
  }

  return true;
}

// TODO(jiacheng): eventually this entire "path_decider" should be retired.
// Before it gets retired, its logics are slightly modified so that everything
// still works well for now.
// 根据路径数据和阻挡障碍物 ID，对路径决策对象中的每个静态障碍物做出决策
bool PathDecider::MakeStaticObstacleDecision(
    const PathData &path_data, const std::string &blocking_obstacle_id,
    PathDecision *const path_decision) {
  // Sanity checks and get important values.
  // 合法性检查
  ACHECK(path_decision);
  const auto &frenet_path = path_data.frenet_frame_path();
  if (frenet_path.empty()) {
    AERROR << "Path is empty.";
    return false;
  }
  const double half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
  const double lateral_radius = half_width + FLAGS_lateral_ignore_buffer;   // FLAGS_lateral_ignore_buffer：3

  // Go through every obstacle and make decisions.
  // 遍历所有障碍物指针
  for (const auto *obstacle : path_decision->obstacles().Items()) {
    // 获取障碍物 ID 和类型（调试打印）
    const std::string &obstacle_id = obstacle->Id();
    const std::string obstacle_type_name =
        PerceptionObstacle_Type_Name(obstacle->Perception().type());
    ADEBUG << "obstacle_id[<< " << obstacle_id << "] type["
           << obstacle_type_name << "]";
    
    // 跳过非静态障碍物或虚拟障碍物
    if (!obstacle->IsStatic() || obstacle->IsVirtual()) {
      continue;
    }

    // - skip decision making for obstacles with IGNORE/STOP decisions already.
    // 如果该障碍物已经被设定为前向和横向都是 IGNORE 决策，则跳过
    if (obstacle->HasLongitudinalDecision() &&
        obstacle->LongitudinalDecision().has_ignore() &&
        obstacle->HasLateralDecision() &&
        obstacle->LateralDecision().has_ignore()) {
      continue;
    }
    
    // 如果已经有 STOP 决策，也跳过
    if (obstacle->HasLongitudinalDecision() &&
        obstacle->LongitudinalDecision().has_stop()) {
      // STOP decision
      continue;
    }

    // - add STOP decision for blocking obstacles.
    // 针对阻挡型障碍物添加 STOP 决策
    // 如果当前障碍物是 blocking_obstacle_id 并且不是车道借道场景
    if (obstacle->Id() == blocking_obstacle_id &&
        !injector_->planning_context()
             ->planning_status()
             .path_decider()
             .is_in_path_lane_borrow_scenario()) {
      // Add stop decision
      ADEBUG << "Blocking obstacle = " << blocking_obstacle_id;
      ObjectDecisionType object_decision;
      *object_decision.mutable_stop() = GenerateObjectStopDecision(*obstacle);
      path_decision->AddLongitudinalDecision("PathDecider/blocking_obstacle",
                                             obstacle->Id(), object_decision);
      continue;
    }

    // - skip decision making for clear-zone obstacles.
    // 忽略 CLEAR 区域的障碍物
    if (obstacle->reference_line_st_boundary().boundary_type() ==
        STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    // 0. IGNORE by default and if obstacle is not in path s at all.
    // 初始化一个 IGNORE 决策
    ObjectDecisionType object_decision;
    object_decision.mutable_ignore();

    // 如果障碍物 s 范围和路径完全没有重叠
    // 则设置为 IGNORE 并跳过
    const auto &sl_boundary = obstacle->PerceptionSLBoundary();
    if (sl_boundary.end_s() < frenet_path.front().s() ||
        sl_boundary.start_s() > frenet_path.back().s()) {
      path_decision->AddLongitudinalDecision("PathDecider/not-in-s",
                                             obstacle->Id(), object_decision);
      path_decision->AddLateralDecision("PathDecider/not-in-s", obstacle->Id(),
                                        object_decision);
      continue;
    }
    
    // 判断障碍物横向位置（是否 STOP 或 NUDGE）
    // 获取路径上与障碍物最近的点 l 值；min_nudge_l 是“会撞上”的横向判定距离
    const auto frenet_point = frenet_path.GetNearestPoint(sl_boundary);
    const double curr_l = frenet_point.l();
    double min_nudge_l = half_width + config_.static_obstacle_buffer() / 2.0;
    
    if (curr_l - lateral_radius > sl_boundary.end_l() ||
        curr_l + lateral_radius < sl_boundary.start_l()) {
      // 障碍物在横向上离得太远
      // 1. IGNORE if laterally too far away.
       // 条件 1：障碍物横向离得很远，IGNORE
      path_decision->AddLateralDecision("PathDecider/not-in-l", obstacle->Id(),
                                        object_decision);
    } else if (sl_boundary.end_l() >= curr_l - min_nudge_l &&
               sl_boundary.start_l() <= curr_l + min_nudge_l) {
      // 若障碍物和路径横向重叠范围超过最小避让范围
      // 2. STOP if laterally too overlapping.
      // 条件 2：障碍物横向重叠，STOP
      *object_decision.mutable_stop() = GenerateObjectStopDecision(*obstacle);
      
      // 如果当前 STOP 可以合并为主停车决策，则添加，否则设为 IGNORE
      if (path_decision->MergeWithMainStop(
              object_decision.stop(), obstacle->Id(),
              reference_line_info_->reference_line(),
              reference_line_info_->AdcSlBoundary())) {
        path_decision->AddLongitudinalDecision("PathDecider/nearest-stop",
                                               obstacle->Id(), object_decision);
      } else {
        ObjectDecisionType object_decision;
        object_decision.mutable_ignore();
        path_decision->AddLongitudinalDecision("PathDecider/not-nearest-stop",
                                               obstacle->Id(), object_decision);
      }
      AINFO << "Add stop decision for static obs " << obstacle->Id()
            << "start l" << sl_boundary.start_l() << "end l"
            << sl_boundary.end_l() << "curr_l" << curr_l << "min_nudge_l"
            << min_nudge_l;
    } else {
      // 3. NUDGE if laterally very close.
      // 条件 3：距离很近但不重叠，NUDGE
      if (sl_boundary.end_l() < curr_l - min_nudge_l) {  // &&
        // sl_boundary.end_l() > curr_l - min_nudge_l - 0.3) {
        // LEFT_NUDGE
        ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
        object_nudge_ptr->set_type(ObjectNudge::LEFT_NUDGE);
        object_nudge_ptr->set_distance_l(config_.static_obstacle_buffer());
        path_decision->AddLateralDecision("PathDecider/left-nudge",
                                          obstacle->Id(), object_decision);
      } else if (sl_boundary.start_l() > curr_l + min_nudge_l) {  // &&
        // sl_boundary.start_l() < curr_l + min_nudge_l + 0.3) {
        // RIGHT_NUDGE
        ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
        object_nudge_ptr->set_type(ObjectNudge::RIGHT_NUDGE);
        object_nudge_ptr->set_distance_l(-config_.static_obstacle_buffer());
        path_decision->AddLateralDecision("PathDecider/right-nudge",
                                          obstacle->Id(), object_decision);
      }
    }
  }

  return true;
}

ObjectStop PathDecider::GenerateObjectStopDecision(
    const Obstacle &obstacle) const {
  ObjectStop object_stop;
  // 把停车点设置为车辆前缘刚好到达该停止点
  double stop_distance = obstacle.MinRadiusStopDistance(
      VehicleConfigHelper::GetConfig().vehicle_param());
  // 设置停车的原因码为：由于障碍物停车（STOP_REASON_OBSTACLE）
  object_stop.set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);
  // 设置停车点相对于障碍物的 s 向距离。注意是负值，意味着停车点位于障碍物前方（在车辆前进方向上提前停车）
  object_stop.set_distance_s(-stop_distance);

  // 计算停车点在参考线上的 s 坐标：障碍物的前沿 s 减去安全停车距离
  const double stop_ref_s =
      obstacle.PerceptionSLBoundary().start_s() - stop_distance;
  // 在参考线上获取 stop_ref_s 位置对应的 x, y, heading 世界坐标
  const auto stop_ref_point =
      reference_line_info_->reference_line().GetReferencePoint(stop_ref_s);
  // 将停止点的 x 和 y 坐标设置为参考线上的对应位置
  object_stop.mutable_stop_point()->set_x(stop_ref_point.x());
  object_stop.mutable_stop_point()->set_y(stop_ref_point.y());
  // 设置停车点的朝向（heading），与参考线保持一致
  object_stop.set_stop_heading(stop_ref_point.heading());
  return object_stop;
}

// 忽略在车辆后方的非静态障碍物，避免对它们进行无意义的决策处理
bool PathDecider::IgnoreBackwardObstacle(PathDecision *const path_decision) {
  // 获取车辆起始位置 s 值
  double adc_start_s = reference_line_info_->AdcSlBoundary().start_s();

  // 遍历路径决策中的所有障碍物
  for (const auto *obstacle : path_decision->obstacles().Items()) {
    // 过滤掉静态或虚拟障碍物
    if (obstacle->IsStatic() || obstacle->IsVirtual()) {
      continue;
    }

    // 判断障碍物是否在车辆后方
    if (obstacle->Obstacle::PerceptionSLBoundary().end_s() < adc_start_s) {
      // 为后方障碍物添加 IGNORE 决策
      ObjectDecisionType object_decision;
      object_decision.mutable_ignore();
      path_decision->AddLongitudinalDecision(
          "PathDecider/ignore-backward-obstacle", obstacle->Id(),
          object_decision);
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
