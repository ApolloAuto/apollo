/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/scenarios/lane_follow/lane_follow_stage.h"
// 引入 C++ 标准库中的工具函数（如 std::move, std::swap
#include <utility>
// 引入 Apollo 的日志宏，如 ADEBUG, AINFO, AERROR，用于输出调试或错误信息
#include "cyber/common/log.h"
// 用于获取系统时间，计算每个规划任务（Task）的执行耗时（性能分析）
#include "cyber/time/clock.h"
// 提供数学工具函数（如角度归一化、插值等），可能被 Task 内部调用
#include "modules/common/math/math_utils.h"
// 用于构造 TrajectoryPoint、SLPoint 等几何点对象
#include "modules/common/util/point_factory.h"
// 字符串处理工具（如格式化），可能用于日志输出
#include "modules/common/util/string_util.h"
// 获取车辆当前状态（位置、速度、加速度、航向等）
#include "modules/common/vehicle_state/vehicle_state_provider.h"
// 高精地图接口，用于查询车道、路口、交通标志等信息
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
// 自车（Ego Vehicle）相关信息封装
#include "modules/planning/planning_base/common/ego_info.h"
// Frame 是规划模块的核心数据结构，包含感知、定位、地图、参考线等所有上下文信息
#include "modules/planning/planning_base/common/frame.h"
// 用于生成速度剖面（fallback 轨迹等）
#include "modules/planning/planning_base/common/speed_profile_generator.h"
// 引入规划模块的运行时配置参数（如 FLAGS_enable_record_debug, FLAGS_enable_trajectory_check）
#include "modules/planning/planning_base/gflags/planning_gflags.h"
// 轨迹合法性检查器（验证曲率、加速度、速度是否超限）
#include "modules/planning/planning_base/math/constraint_checker/constraint_checker.h"
// Task 基类定义，LaneFollowStage 会依次执行一系列 Task（如 PathDecider、SpeedDecider）
#include "modules/planning/planning_interface_base/task_base/task.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::SLPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::util::PointFactory;
using apollo::cyber::Clock;

namespace {
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

/// @brief 将障碍物的决策信息记录到调试日志中，便于可视化分析
/// @param reference_line_info 
void LaneFollowStage::RecordObstacleDebugInfo(
    ReferenceLineInfo* reference_line_info) {
  // 如果未启用调试记录（通过 GFlag 控制），则直接返回
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }
  // 获取可修改的调试信息对象指针
  auto ptr_debug = reference_line_info->mutable_debug();
  // 获取路径决策器对所有障碍物做出的决策（如避让、跟随、忽略）
  const auto path_decision = reference_line_info->path_decision();
  // 遍历所有障碍物
  for (const auto obstacle : path_decision->obstacles().Items()) {
    // 将障碍物 ID 和其在 SL 坐标系下的边界（s: 纵向, l: 横向）拷贝到调试结构中
    auto obstacle_debug = ptr_debug->mutable_planning_data()->add_obstacle();
    obstacle_debug->set_id(obstacle->Id());
    obstacle_debug->mutable_sl_boundary()->CopyFrom(
        obstacle->PerceptionSLBoundary());

    // 每个障碍物可能被多个 Decider 处理（如 PathDecider、SpeedDecider），这里检查标签和决策数量是否一致，不一致说明逻辑错误
    const auto& decider_tags = obstacle->decider_tags();
    const auto& decisions = obstacle->decisions();
    if (decider_tags.size() != decisions.size()) {
      AERROR << "decider_tags size: " << decider_tags.size()
             << " different from decisions size:" << decisions.size();
    }
    // 将每个 Decider 的名称和对应决策（如“横向忽略，纵向停车”）记录到调试信息中
    for (size_t i = 0; i < decider_tags.size(); ++i) {
      auto decision_tag = obstacle_debug->add_decision_tag();
      decision_tag->set_decider_tag(decider_tags[i]);
      decision_tag->mutable_decision()->CopyFrom(decisions[i]);
    }
  }
}

// 处理所有候选参考线，选择一条可行驶的路径
StageResult LaneFollowStage::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  // 如果没有参考线（如地图缺失），直接返回 FINISHED（无事可做）
  if (frame->reference_line_info().empty()) {
    return StageResult(StageStatusType::FINISHED);
  }
  // 标志位，记录是否已找到可行驶的参考线
  bool has_drivable_reference_line = false;

  ADEBUG << "Number of reference lines:\t"
         << frame->mutable_reference_line_info()->size();
  
  // 遍历每条参考线（通常是 [当前车道, 左变道, 右变道]）
  unsigned int count = 0;
  StageResult result;
  // 遍历所有的参考线，直到找到可用来规划的参考线后退出
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    // TODO(SHU): need refactor
    // 这是一个冗余的安全检查（因为 range-for 不会越界），可能是历史遗留代码，可忽略
    if (count++ == frame->mutable_reference_line_info()->size()) {
      break;
    }
    ADEBUG << "No: [" << count << "] Reference Line.";
    ADEBUG << "IsChangeLanePath: " << reference_line_info.IsChangeLanePath();
  // 找到可用来行驶的参考线，退出循环
  // 一旦找到可行驶路径，立即退出循环，后续参考线标记为不可用
    if (has_drivable_reference_line) {
      reference_line_info.SetDrivable(false);
      break;
    }
  // 执行LaneFollow的规划
  // 在当前参考线上执行完整的规划流程（路径+速度）
    result =
        PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);
  // 判断规划结果是否OK
  // 如果规划成功（无 Task 错误）
    if (!result.HasError()) {
      // 不是变道的参考线，则为可行驶参考线
      // 如果是直行路径，直接接受，继续下一轮（实际会因 has_drivable...=true 而退出
      if (!reference_line_info.IsChangeLanePath()) {
        ADEBUG << "reference line is NOT lane change ref.";
        has_drivable_reference_line = true;
        continue;
      }

      // 如果是变道路径，只有当总成本 低于 10.0 才接受
      // 如果是变道参考线，则对参考线的cost进行判断
      if (reference_line_info.Cost() < kStraightForwardLineCost) {
        // If the path and speed optimization succeed on target lane while
        // under smart lane-change or IsClearToChangeLane under older version
        has_drivable_reference_line = true;
        reference_line_info.SetDrivable(true);
      } else {
        // 如果规划失败（Task 报错），标记为不可用
        reference_line_info.SetDrivable(false);
        ADEBUG << "\tlane change failed";
      }
    } else {
      reference_line_info.SetDrivable(false);
    }
  }
  // 如果找到可行驶路径 → 返回 RUNNING（正常）
  return has_drivable_reference_line
             ? result.SetStageStatus(StageStatusType::RUNNING)
             : result.SetStageStatus(StageStatusType::ERROR);
}
/// @brief 
/// @param planning_start_point 
/// @param frame 
/// @param reference_line_info 
/// @return 
StageResult LaneFollowStage::PlanOnReferenceLine(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
 // 判断是否有lanechange意图，如果否增加当前参考线的cost？有点疑问，增加了变道的可能
 // 为直行路径增加 10.0 成本（但不影响其优先级，仅用于变道比较）
  if (!reference_line_info->IsChangeLanePath()) {
    reference_line_info->AddCost(kStraightForwardLineCost);
  }
  ADEBUG << "planning start point:" << planning_start_point.DebugString();
  ADEBUG << "Current reference_line_info is IsChangeLanePath: "
         << reference_line_info->IsChangeLanePath();
  // 顺序执行stage中的任务
  // 依次执行 Task 列表（如 PATH_BOUNDS_DECIDER → PATH_DECIDER → SPEED_DECIDER）
  StageResult ret;
  for (auto task : task_list_) {
    // 执行每个 Task，若失败则中断
    const double start_timestamp = Clock::NowInSeconds();
    // 性能计时
    const auto start_planning_perf_timestamp =
        std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
  // 执行每个task的具体逻辑
  //依次调用task_list中的task
    ret.SetTaskStatus(task->Execute(frame, reference_line_info));
    // 记录耗时
    const double end_timestamp = Clock::NowInSeconds();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
    ADEBUG << "after task[" << task->Name()
           << "]:" << reference_line_info->PathSpeedDebugString();
    ADEBUG << task->Name() << " time spend: " << time_diff_ms << " ms.";
    RecordDebugInfo(reference_line_info, task->Name(), time_diff_ms);

    const auto end_planning_perf_timestamp =
        std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    const auto plnning_perf_ms =
        (end_planning_perf_timestamp - start_planning_perf_timestamp) * 1000;
    AINFO << "Planning Perf: task name [" << task->Name() << "], "
          << plnning_perf_ms << " ms.";
  // 如果task执行失败，退出task执行序列，并且记录失败信息
    if (ret.IsTaskError()) {
      AERROR << "Failed to run tasks[" << task->Name()
             << "], Error message: " << ret.GetTaskStatus().error_message();
      break;
    }

    // TODO(SHU): disable reference line order changes for now
    // updated reference_line_info, because it is changed in
    // lane_change_decider by PrioritizeChangeLane().
    // reference_line_info = &frame->mutable_reference_line_info()->front();
    // ADEBUG << "Current reference_line_info is IsChangeLanePath: "
    //        << reference_line_info->IsChangeLanePath();
  }
  // 记录障碍物调试信息
  RecordObstacleDebugInfo(reference_line_info);

  // check path and speed results for path or speed fallback
  reference_line_info->set_trajectory_type(ADCTrajectory::NORMAL);
  // 如果task执行失败，则使用备用的规划轨迹
  // Task 失败时，执行回退策略（如生成安全停车轨迹）
  if (ret.IsTaskError()) {
    fallback_task_->Execute(frame, reference_line_info);
  }
  // 对规划的轨迹进行合成，如果合成失败，返回失败状态
  // 将规划出的路径（s-l）和速度（s-t）合并为时空轨迹（x-y-t）。失败则报错
  DiscretizedTrajectory trajectory;
  if (!reference_line_info->CombinePathAndSpeedProfile(
          planning_start_point.relative_time(),
          planning_start_point.path_point().s(), &trajectory)) {
    const std::string msg = "Fail to aggregate planning trajectory.";
    AERROR << msg;
    return ret.SetStageStatus(StageStatusType::ERROR, msg);
  }

  // determine if there is a destination on reference line.
  // 查找是否在本参考线上有终点停车点（用于区分普通障碍物停车）
  double dest_stop_s = -1.0;
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle->LongitudinalDecision().has_stop() &&
        obstacle->LongitudinalDecision().stop().reason_code() ==
            STOP_REASON_DESTINATION) {
      SLPoint dest_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
                                  reference_line_info->reference_line());
      dest_stop_s = dest_sl.s();
    }
  }
// 增加障碍物的代价
// 如果路径上有一个非终点的静态障碍物需要停车，且距离终点较近（<20m），就增加 1000 成本，降低该参考线优先级（避免选一条“走几步就停”的路）
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle->IsVirtual()) {
      continue;
    }
    if (!obstacle->IsStatic()) {
      continue;
    }
    if (obstacle->LongitudinalDecision().has_stop()) {
      bool add_stop_obstacle_cost = false;
      if (dest_stop_s < 0.0) {
        add_stop_obstacle_cost = true;
      } else {
        SLPoint stop_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
                                    reference_line_info->reference_line());
        if (stop_sl.s() < dest_stop_s &&
            (dest_stop_s - reference_line_info->AdcSlBoundary().end_s()) <
                20.0) {
          add_stop_obstacle_cost = true;
        }
      }
      if (add_stop_obstacle_cost) {
        static constexpr double kReferenceLineStaticObsCost = 1e3;
        reference_line_info->AddCost(kReferenceLineStaticObsCost);
      }
    }
  }
  // 启用轨迹检查时，验证轨迹是否满足车辆动力学约束
  if (FLAGS_enable_trajectory_check) {
    if (ConstraintChecker::ValidTrajectory(trajectory) !=
        ConstraintChecker::Result::VALID) {
      const std::string msg = "Current planning trajectory is not valid.";
      AERROR << msg;
      return ret.SetStageStatus(StageStatusType::ERROR, msg);
    }
  }
  // 设置最终轨迹，标记为可行驶，返回成功
  reference_line_info->SetTrajectory(trajectory);
  reference_line_info->SetDrivable(true);
  ret.SetStageStatus(StageStatusType::RUNNING);
  return ret;
}

// 将障碍物的停车点（x, y）转换到当前参考线的 SL 坐标系（s: 纵向距离, l: 横向偏移）
SLPoint LaneFollowStage::GetStopSL(const ObjectStop& stop_decision,
                                   const ReferenceLine& reference_line) const {
  SLPoint sl_point;
  reference_line.XYToSL(stop_decision.stop_point(), &sl_point);
  return sl_point;
}

}  // namespace planning
}  // namespace apollo
