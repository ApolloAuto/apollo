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

#include "modules/planning/planning_base/common/obstacle_blocking_analyzer.h"

#include <algorithm>
#include <memory>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/point_factory.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleConfigHelper;
using apollo::hdmap::HDMapUtil;

constexpr double kAdcDistanceThreshold = 35.0;  // unit: m
constexpr double kObstaclesDistanceThreshold = 15.0;
/// @brief 判断一个障碍物（obstacle）是否为不可移动的障碍物
/// @param reference_line_info 包含参考线信息的对象
/// @param obstacle 当前障碍物的对象
/// @return 
bool IsNonmovableObstacle(const ReferenceLineInfo& reference_line_info,
                          const Obstacle& obstacle) {
  // Obstacle is far away.
  // 自动驾驶车辆的当前边界，用于后续判断障碍物的位置
  const SLBoundary& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  // 如果障碍物的起始位置（start_s()）距离参考线边界的结束位置（end_s()）加上一个阈值（kAdcDistanceThreshold）还远，那么认为该障碍物太远，系统无法确定其状态，返回 false
  if (obstacle.PerceptionSLBoundary().start_s() >
      adc_sl_boundary.end_s() + kAdcDistanceThreshold) {
    ADEBUG << " - It is too far ahead and we are not so sure of its status.";
    return false;
  }

  // Obstacle is parked obstacle.
  if (IsParkedVehicle(reference_line_info.reference_line(), &obstacle)) {
    ADEBUG << "It is Parked and NON-MOVABLE.";
    return true;
  }

  // Obstacle is blocked by others too.
  // 遍历参考线信息中所有的障碍物，检查是否有其他障碍物可能阻挡当前障碍物
  for (const auto* other_obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
  // 如果当前障碍物是正在检查的障碍物自身，跳过此次循环
    if (other_obstacle->Id() == obstacle.Id()) {
      continue;
    }
  // 如果其他障碍物是虚拟障碍物（如由算法模拟出来的障碍物），则跳过此次循环
    if (other_obstacle->IsVirtual()) {
      continue;
    }
  // 如果其他障碍物不是车辆类型的障碍物，则跳过此次循环
    if (other_obstacle->Perception().type() !=
        apollo::perception::PerceptionObstacle::VEHICLE) {
      continue;
    }
    // 获取当前检查的障碍物和其他障碍物的感知边界
    const auto& other_boundary = other_obstacle->PerceptionSLBoundary();
    const auto& this_boundary = obstacle.PerceptionSLBoundary();
    // 如果其他障碍物的左右边界（start_l() 和 end_l()）没有与当前障碍物的边界重叠，即它们不在相邻的车道内，则认为它们不构成阻挡，跳过此次循环
    if (other_boundary.start_l() > this_boundary.end_l() ||
        other_boundary.end_l() < this_boundary.start_l()) {
      // not blocking the backside vehicle
      continue;
    }
    // 计算当前障碍物与其他障碍物的纵向距离（delta_s）。如果这个距离小于零或大于一个预设的阈值（kObstaclesDistanceThreshold），则认为其他障碍物不构成阻挡，跳过此次循环
    double delta_s = other_boundary.start_s() - this_boundary.end_s();
    if (delta_s < 0.0 || delta_s > kObstaclesDistanceThreshold) {
      continue;
    }
    // 如果以上条件都满足，说明当前障碍物被其他障碍物阻挡
    return false;
  }
  ADEBUG << "IT IS NON-MOVABLE!";
  // 如果没有其他障碍物阻挡，且障碍物满足不可移动条件，输出日志并返回 true，表示当前障碍物不可移动
  return true;
}

// This is the side-pass condition for every obstacle.
// TODO(all): if possible, transform as many function parameters into GFLAGS.
bool IsBlockingObstacleToSidePass(const Frame& frame, const Obstacle* obstacle,
                                  double block_obstacle_min_speed,
                                  double min_front_sidepass_distance,
                                  bool enable_obstacle_blocked_check) {
  // Get the necessary info.
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& reference_line = reference_line_info.reference_line();
  const SLBoundary& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  const PathDecision& path_decision = reference_line_info.path_decision();
  ADEBUG << "Evaluating Obstacle: " << obstacle->Id();

  // Obstacle is virtual.
  if (obstacle->IsVirtual()) {
    ADEBUG << " - It is virtual.";
    return false;
  }

  // Obstacle is moving.
  if (!obstacle->IsStatic() || obstacle->speed() > block_obstacle_min_speed) {
    ADEBUG << " - It is non-static.";
    return false;
  }

  // Obstacle is behind ADC.
  if (obstacle->PerceptionSLBoundary().start_s() <= adc_sl_boundary.end_s()) {
    ADEBUG << " - It is behind ADC.";
    return false;
  }

  // Obstacle is far away.
  static constexpr double kAdcDistanceSidePassThreshold = 15.0;
  if (obstacle->PerceptionSLBoundary().start_s() >
      adc_sl_boundary.end_s() + kAdcDistanceSidePassThreshold) {
    ADEBUG << " - It is too far ahead.";
    return false;
  }

  // Obstacle is too close.
  if (adc_sl_boundary.end_s() + min_front_sidepass_distance >
      obstacle->PerceptionSLBoundary().start_s()) {
    ADEBUG << " - It is too close to side-pass.";
    return false;
  }

  // Obstacle is not blocking our path.
  if (!IsBlockingDrivingPathObstacle(reference_line, obstacle)) {
    ADEBUG << " - It is not blocking our way.";
    return false;
  }

  // Obstacle is blocked by others too.
  if (enable_obstacle_blocked_check &&
      !IsParkedVehicle(reference_line, obstacle)) {
    for (const auto* other_obstacle : path_decision.obstacles().Items()) {
      if (other_obstacle->Id() == obstacle->Id()) {
        continue;
      }
      if (other_obstacle->IsVirtual()) {
        continue;
      }
      if (other_obstacle->PerceptionSLBoundary().start_l() >
              obstacle->PerceptionSLBoundary().end_l() ||
          other_obstacle->PerceptionSLBoundary().end_l() <
              obstacle->PerceptionSLBoundary().start_l()) {
        // not blocking the backside vehicle
        continue;
      }
      double delta_s = other_obstacle->PerceptionSLBoundary().start_s() -
                       obstacle->PerceptionSLBoundary().end_s();
      if (delta_s < 0.0 || delta_s > kAdcDistanceThreshold) {
        continue;
      }

      // TODO(All): Fix the segmentation bug for large vehicles, otherwise
      // the follow line will be problematic.
      ADEBUG << " - It is blocked by others, too.";
      return false;
    }
  }

  ADEBUG << "IT IS BLOCKING!";
  return true;
}

double GetDistanceBetweenADCAndObstacle(const Frame& frame,
                                        const Obstacle* obstacle) {
  const auto& reference_line_info = frame.reference_line_info().front();
  const SLBoundary& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  double distance_between_adc_and_obstacle =
      obstacle->PerceptionSLBoundary().start_s() - adc_sl_boundary.end_s();
  return distance_between_adc_and_obstacle;
}

bool IsBlockingDrivingPathObstacle(const ReferenceLine& reference_line,
                                   const Obstacle* obstacle) {
  const double driving_width =
      reference_line.GetDrivingWidth(obstacle->PerceptionSLBoundary());
  const double adc_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width();
  ADEBUG << " (driving width = " << driving_width
         << ", adc_width = " << adc_width << ")";
  if (driving_width > adc_width + FLAGS_static_obstacle_nudge_l_buffer +
                          FLAGS_side_pass_driving_width_l_buffer) {
    // TODO(jiacheng): make this a GFLAG:
    // side_pass_context_.scenario_config_.min_l_nudge_buffer()
    ADEBUG << "It is NOT blocking our path.";
    return false;
  }

  ADEBUG << "It is blocking our path.";
  return true;
}

bool IsParkedVehicle(const ReferenceLine& reference_line,
                     const Obstacle* obstacle) {
  if (!FLAGS_enable_scenario_side_pass_multiple_parked_obstacles) {
    return false;
  }
  double road_left_width = 0.0;
  double road_right_width = 0.0;
  double max_road_right_width = 0.0;
  reference_line.GetRoadWidth(obstacle->PerceptionSLBoundary().start_s(),
                              &road_left_width, &road_right_width);
  max_road_right_width = road_right_width;
  reference_line.GetRoadWidth(obstacle->PerceptionSLBoundary().end_s(),
                              &road_left_width, &road_right_width);
  max_road_right_width = std::max(max_road_right_width, road_right_width);
  bool is_at_road_edge = std::abs(obstacle->PerceptionSLBoundary().start_l()) >
                         max_road_right_width - 0.1;

  std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
  auto obstacle_box = obstacle->PerceptionBoundingBox();
  HDMapUtil::BaseMapPtr()->GetLanes(
      common::util::PointFactory::ToPointENU(obstacle_box.center().x(),
                                             obstacle_box.center().y()),
      std::min(obstacle_box.width(), obstacle_box.length()), &lanes);
  bool is_on_parking_lane = false;
  if (lanes.size() == 1 &&
      lanes.front()->lane().type() == apollo::hdmap::Lane::PARKING) {
    is_on_parking_lane = true;
  }

  bool is_parked = is_on_parking_lane || is_at_road_edge;
  return is_parked && obstacle->IsStatic();
}

}  // namespace planning
}  // namespace apollo
