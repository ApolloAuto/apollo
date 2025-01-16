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

#include "modules/planning/tasks/open_space_pre_stop_decider/open_space_pre_stop_decider.h"

#include <memory>
#include <string>
#include <vector>

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;

bool OpenSpacePreStopDecider::Init(
    const std::string& config_dir, const std::string& name,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (!Decider::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  bool res = Decider::LoadConfig<OpenSpacePreStopDeciderConfig>(&config_);
  AINFO << "Load config:" << config_.DebugString();
  return res;
}
/// @brief 
/// @param frame 
/// @param reference_line_info 
/// @return 
Status OpenSpacePreStopDecider::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  // 检查输入指针是否为空
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);
  // 存储目标停车位置的坐标
  double target_s = 0.0;
  const auto& stop_type = config_.stop_type();
  switch (stop_type) {
    case OpenSpacePreStopDeciderConfig::PARKING:
       // 检查停车位的预停车条件
      if (!CheckParkingSpotPreStop(frame, reference_line_info, &target_s)) {
        const std::string msg = "Checking parking spot pre stop fails";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
      // 设置虚拟墙
      SetParkingSpotStopFence(target_s, frame, reference_line_info);
      break;
    case OpenSpacePreStopDeciderConfig::PULL_OVER:
      if (!CheckPullOverPreStop(frame, reference_line_info, &target_s)) {
        const std::string msg = "Checking pull over pre stop fails";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
      SetPullOverStopFence(target_s, frame, reference_line_info);
      break;
    default:
      const std::string msg = "This stop type not implemented";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}
/// @brief 
/// @param frame 
/// @param reference_line_info 
/// @param target_s 指向目标 s 坐标的指针，函数会将计算出的目标停车位置的 s 坐标存储在此变量中
/// @return 
bool OpenSpacePreStopDecider::CheckPullOverPreStop(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    double* target_s) {
  // 目标 s 坐标的初始值
  *target_s = 0.0;
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  if (pull_over_status.has_position() && pull_over_status.position().has_x() &&
      pull_over_status.position().has_y()) {
    common::SLPoint pull_over_sl;
    const auto& reference_line = reference_line_info->reference_line();
    reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);
    *target_s = pull_over_sl.s();
  }
  return true;
}
/// @brief 检查目标停车位是否存在，并计算该停车位在参考线上的位置
/// @param frame 
/// @param reference_line_info 
/// @param target_s 
/// @return 
bool OpenSpacePreStopDecider::CheckParkingSpotPreStop(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    double* target_s) {
  // 获取目标停车位的 ID
  const auto& target_parking_spot_id =
      frame->open_space_info().target_parking_spot_id();
  // 获取参考线信息，并进一步获取与该参考线相关的地图路径（map_path），这将用于定位停车位的位置
  const auto& nearby_path = reference_line_info->reference_line().map_path();
  if (target_parking_spot_id.empty()) {
    AERROR << "no target parking spot id found when setting pre stop fence";
    return false;
  }
  // 目标停车区域的 s 坐标，初始化为 0.0
  double target_area_center_s = 0.0;
  // 是否找到目标停车区域，初始化为 false
  bool target_area_found = false;
  // 获取与参考线相关的停车位重叠区域
  const auto& parking_space_overlaps = nearby_path.parking_space_overlaps();
  // 存储目标停车位的信息
  ParkingSpaceInfoConstPtr target_parking_spot_ptr;
  // 访问地图数据
  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  // 遍历所有停车位重叠区域（parking_space_overlaps）
  for (const auto& parking_overlap : parking_space_overlaps) {
    if (parking_overlap.object_id == target_parking_spot_id) {
      // TODO(Jinyun) parking overlap s are wrong on map, not usable
      // target_area_center_s =
      //     (parking_overlap.start_s + parking_overlap.end_s) / 2.0;
      hdmap::Id id;
      id.set_id(parking_overlap.object_id);
      target_parking_spot_ptr = hdmap->GetParkingSpaceById(id);
      // 获取目标停车位的四个顶点坐标（左下、右下、右上、左上）
      Vec2d left_bottom_point =
          target_parking_spot_ptr->polygon().points().at(0);
      Vec2d right_bottom_point =
          target_parking_spot_ptr->polygon().points().at(1);
      Vec2d right_up_point = target_parking_spot_ptr->polygon().points().at(2);
      Vec2d left_up_point = target_parking_spot_ptr->polygon().points().at(3);
      // 计算停车位的中心点坐标，即四个顶点坐标的平均值
      Vec2d center_point = (left_bottom_point + right_bottom_point +
                            right_up_point + left_up_point) /
                           4.0;
      // 获取停车位中心点 center_point 在参考线中的最近点，并将对应的 s 坐标存储在 target_area_center_s
      double center_l;
      nearby_path.GetNearestPoint(center_point, &target_area_center_s,
                                  &center_l);
      target_area_found = true;
    }
  }

  if (!target_area_found) {
    AERROR << "no target parking spot found on reference line";
    return false;
  }
  *target_s = target_area_center_s;
  return true;
}
/// @brief 根据目标停车位置和车辆状态计算停车线的位置，并根据情况设置停车围栏。如果目标停车位置距离车辆的前端较远，停车线会设置在目标位置之前；
//       如果目标停车位置距离车辆前端较近，停车线则会设置在目标位置之后。停车线计算完成后，函数会更新停车围栏的位置并构建停车决策
/// @param target_s 
/// @param frame 
/// @param reference_line_info 
void OpenSpacePreStopDecider::SetParkingSpotStopFence(
    const double target_s, Frame* const frame,
    ReferenceLineInfo* const reference_line_info) {
  // 获取与当前参考线相关的地图路径（map_path），表示车辆所处的道路路径
  const auto& nearby_path = reference_line_info->reference_line().map_path();
  // 获取自动驾驶车辆（ADC）的前端边界的 s 坐标
  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  // 获取车辆的当前状态（vehicle_state），包括速度、加速度等信息
  const VehicleState& vehicle_state = frame->vehicle_state();
  // 计算出的停车线的 s 坐标
  double stop_line_s = 0.0;
  // 配置中获取停车线与目标停车位置之间的距离
  double stop_distance_to_target = config_.stop_distance_to_target();
  // 判断车辆是否几乎静止
  double static_linear_velocity_epsilon = 1.0e-2;
  // 停车缓冲区，用于设置停车线的容忍范围
  static constexpr double kStopBuffer = 0.2;
  // 检查 stop_distance_to_target 是否大于或等于 1.0e-8
  CHECK_GE(stop_distance_to_target, 1.0e-8);
  // 目标停车位置与车辆前端边界之间的偏移量
  double target_vehicle_offset = target_s - adc_front_edge_s;
  if (target_vehicle_offset > stop_distance_to_target) {
    stop_line_s = target_s - stop_distance_to_target;
  } else if (target_vehicle_offset < stop_distance_to_target - kStopBuffer) {
    stop_line_s = target_s + stop_distance_to_target;
  } else if (target_vehicle_offset < -stop_distance_to_target) {
    // 如果没有设置 "立即停车" 的标志
    if (!frame->open_space_info().pre_stop_rightaway_flag()) {
      // TODO(Jinyun) Use constant comfortable deacceleration rather than
      // distance by config to set stop fence
      stop_line_s = adc_front_edge_s + config_.rightaway_stop_distance();
      // 车辆几乎静止，停车线设置为车辆前端的位置
      if (std::abs(vehicle_state.linear_velocity()) <
          static_linear_velocity_epsilon) {
        stop_line_s = adc_front_edge_s;
      }
      // 将计算出的停车线位置转换为参考线上的平滑点，并更新到 frame 的开放空间信息中
      *(frame->mutable_open_space_info()->mutable_pre_stop_rightaway_point()) =
          nearby_path.GetSmoothPoint(stop_line_s);
      // 设置 "立即停车" 标志为 true，表示已经设置了停车围栏
      frame->mutable_open_space_info()->set_pre_stop_rightaway_flag(true);
    } else {
      // 如果 "立即停车" 标志已经设置，那么获取前一次计算的停车点，找到该点在参考线上的最近点并更新停车线位置
      double stop_point_s = 0.0;
      double stop_point_l = 0.0;
      nearby_path.GetNearestPoint(
          frame->open_space_info().pre_stop_rightaway_point(), &stop_point_s,
          &stop_point_l);
      stop_line_s = stop_point_s;
    }
  }
  // 定义停车围栏的标识符 stop_wall_id，使用常量 OPEN_SPACE_STOP_ID 作为唯一标识符
  const std::string stop_wall_id = OPEN_SPACE_STOP_ID;
  // 存储需要等待的障碍物标识符
  std::vector<std::string> wait_for_obstacles;
  // 将计算得到的停车线位置（stop_line_s）更新到 frame 的开放空间信息中，表示停车围栏的 s 坐标
  frame->mutable_open_space_info()->set_open_space_pre_stop_fence_s(
      stop_line_s);
  // 创建停车决策
  util::BuildStopDecision(stop_wall_id, stop_line_s, 0.0,
                          StopReasonCode::STOP_REASON_PRE_OPEN_SPACE_STOP,
                          wait_for_obstacles, "OpenSpacePreStopDecider", frame,
                          reference_line_info);
}

void OpenSpacePreStopDecider::SetPullOverStopFence(
    const double target_s, Frame* const frame,
    ReferenceLineInfo* const reference_line_info) {
  const auto& nearby_path = reference_line_info->reference_line().map_path();
  // 车辆前端在参考线上的位置
  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  // 获取当前车辆的状态信息，包括速度、加速度等
  const VehicleState& vehicle_state = frame->vehicle_state();
  double stop_line_s = 0.0;
  double stop_distance_to_target = config_.stop_distance_to_target();
  double static_linear_velocity_epsilon = 1.0e-2;
  CHECK_GE(stop_distance_to_target, 1.0e-8);
  double target_vehicle_offset = target_s - adc_front_edge_s;
  if (target_vehicle_offset > stop_distance_to_target) {
    stop_line_s = target_s - stop_distance_to_target;
  } else {// 如果尚未设置 "右侧停车优先" 标志
    if (!frame->open_space_info().pre_stop_rightaway_flag()) {
      // TODO(Jinyun) Use constant comfortable deacceleration rather than
      // distance by config to set stop fence
      stop_line_s = adc_front_edge_s + config_.rightaway_stop_distance();
      if (std::abs(vehicle_state.linear_velocity()) <
          static_linear_velocity_epsilon) {
        stop_line_s = adc_front_edge_s;
      }
      *(frame->mutable_open_space_info()->mutable_pre_stop_rightaway_point()) =
          nearby_path.GetSmoothPoint(stop_line_s);
      frame->mutable_open_space_info()->set_pre_stop_rightaway_flag(true);
    } else {
      double stop_point_s = 0.0;
      double stop_point_l = 0.0;
      nearby_path.GetNearestPoint(
          frame->open_space_info().pre_stop_rightaway_point(), &stop_point_s,
          &stop_point_l);
      stop_line_s = stop_point_s;
    }
  }

  const std::string stop_wall_id = OPEN_SPACE_STOP_ID;
  std::vector<std::string> wait_for_obstacles;
  frame->mutable_open_space_info()->set_open_space_pre_stop_fence_s(
      stop_line_s);
  util::BuildStopDecision(stop_wall_id, stop_line_s, 0.0,
                          StopReasonCode::STOP_REASON_PRE_OPEN_SPACE_STOP,
                          wait_for_obstacles, "OpenSpacePreStopDecider", frame,
                          reference_line_info);
}
}  // namespace planning
}  // namespace apollo
