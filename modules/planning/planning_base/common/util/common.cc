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

#include "modules/planning/planning_base/common/util/common.h"

namespace apollo {
namespace planning {
namespace util {

using apollo::common::util::WithinBound;

/*
 * @brief: build virtual obstacle of stop wall, and add STOP decision
 */
/// @brief 根据停车线的位置和一些参数创建停车决策，并将该决策添加到参考线的路径决策中
/// @param stop_wall_id 停车围栏的标识符
/// @param stop_line_s 停车线的位置（在参考线上的 s 坐标）
/// @param stop_distance 停车距离，即车辆从停车线到目标停车点的距离
/// @param stop_reason_code 停车的原因代码（例如，停车以避免碰撞等）
/// @param wait_for_obstacles 一个字符串数组，表示在停车前需要等待的障碍物标识符
/// @param decision_tag 决策标签，用于标识这个停车决策
/// @param frame 当前的帧对象，包含车辆的状态和其他信息
/// @param reference_line_info 参考线信息，包含路径和决策
/// @return 
int BuildStopDecision(const std::string& stop_wall_id, const double stop_line_s,
                      const double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag, Frame* const frame,
                      ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // check
  const auto& reference_line = reference_line_info->reference_line();
  // 检查 stop_line_s 是否在参考线的有效范围内
  if (!WithinBound(0.0, reference_line.Length(), stop_line_s)) {
    AERROR << "stop_line_s[" << stop_line_s << "] is not on reference line";
    return 0;
  }

  // create virtual stop wall
  // 此障碍物位于 stop_line_s 位置，并使用 stop_wall_id 作为障碍物的标识符
  const auto* obstacle =
      frame->CreateStopObstacle(reference_line_info, stop_wall_id, stop_line_s);
  // 判断创建的障碍物是否成功
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << stop_wall_id << "]";
    return -1;
  }
  // 将创建的停车围栏障碍物 obstacle 添加到参考线的障碍物列表中，并返回添加后的障碍物对象指针 stop_wall
  const Obstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  // 判断添加障碍物是否成功
  if (!stop_wall) {
    AERROR << "Failed to add obstacle[" << stop_wall_id << "]";
    return -1;
  }

  // build stop decision
  // 计算停车点的位置 stop_s，它位于停车线 stop_line_s 前方 stop_distance 的位置
  const double stop_s = stop_line_s - stop_distance;
  // 该位置的参考点（包含坐标和方向），并计算停车点的朝向 stop_heading
  const auto& stop_point = reference_line.GetReferencePoint(stop_s);
  const double stop_heading =
      reference_line.GetReferencePoint(stop_s).heading();
  // 创建一个 ObjectDecisionType 类型的停车决策对象 stop
  ObjectDecisionType stop;
  auto* stop_decision = stop.mutable_stop();
  // 设置停车原因代码
  stop_decision->set_reason_code(stop_reason_code);
  // 设置停车距离，-stop_distance 表示停车点在停车线前方
  stop_decision->set_distance_s(-stop_distance);
  // 设置停车点的朝向
  stop_decision->set_stop_heading(stop_heading);
  // 设置停车点的坐标
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);
  // 遍历 wait_for_obstacles 列表，将需要等待的障碍物标识符添加到停车决策中
  for (size_t i = 0; i < wait_for_obstacles.size(); ++i) {
    stop_decision->add_wait_for_obstacle(wait_for_obstacles[i]);
  }

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(decision_tag, stop_wall->Id(), stop);

  return 0;
}

int BuildStopDecision(const std::string& stop_wall_id,
                      const std::string& lane_id, const double lane_s,
                      const double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag, Frame* const frame,
                      ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  const auto& reference_line = reference_line_info->reference_line();

  // create virtual stop wall
  const auto* obstacle =
      frame->CreateStopObstacle(stop_wall_id, lane_id, lane_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << stop_wall_id << "]";
    return -1;
  }

  const Obstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create obstacle for: " << stop_wall_id;
    return -1;
  }

  const auto& stop_wall_box = stop_wall->PerceptionBoundingBox();
  if (!reference_line.IsOnLane(stop_wall_box.center())) {
    ADEBUG << "stop point is not on lane. SKIP STOP decision";
    return 0;
  }

  // build stop decision
  auto stop_point = reference_line.GetReferencePoint(
      stop_wall->PerceptionSLBoundary().start_s() - stop_distance);

  ObjectDecisionType stop;
  auto* stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(stop_reason_code);
  stop_decision->set_distance_s(-stop_distance);
  stop_decision->set_stop_heading(stop_point.heading());
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(decision_tag, stop_wall->Id(), stop);

  return 0;
}

}  // namespace util
}  // namespace planning
}  // namespace apollo
