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

#include "modules/planning/planning_base/common/trajectory_stitcher.h"

#include <algorithm>

#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/angle.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_model/vehicle_model.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::common::VehicleModel;
using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
/// @brief 根据当前的车辆状态（vehicle_state）生成一个轨迹点（TrajectoryPoint）
/// @param planning_cycle_time 规划周期 100ms   10hz
/// @param vehicle_state 
/// @return 
TrajectoryPoint TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
    const double planning_cycle_time, const VehicleState& vehicle_state) {
  // 保存生成的轨迹点
  TrajectoryPoint point;
  // 从路径起点累积的距离
  point.mutable_path_point()->set_s(0.0);
  point.mutable_path_point()->set_x(vehicle_state.x());
  point.mutable_path_point()->set_y(vehicle_state.y());
  point.mutable_path_point()->set_z(vehicle_state.z());
  point.mutable_path_point()->set_theta(vehicle_state.heading());
  point.mutable_path_point()->set_kappa(vehicle_state.kappa());
  point.set_v(vehicle_state.linear_velocity());
  point.set_a(vehicle_state.linear_acceleration());
  // 表示此轨迹点在当前规划周期中的位置
  point.set_relative_time(planning_cycle_time);
  return point;
}
/// @brief 根据车辆的当前状态（vehicle_state）和规划周期时间（planning_cycle_time），决定是使用当前车辆状态还是预测的车辆状态来生成新的轨迹点
/// @param planning_cycle_time 
/// @param vehicle_state 
/// @return 
std::vector<TrajectoryPoint>
TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const double planning_cycle_time, const VehicleState& vehicle_state) {
  TrajectoryPoint reinit_point;
  static constexpr double kEpsilon_v = 0.1;
  static constexpr double kEpsilon_a = 0.4;
  // TODO(Jinyun/Yu): adjust kEpsilon if corrected IMU acceleration provided
  // 如果当前车速小于0.1，且加速度小于0.4，则直接当前车辆状态作为规划起点
  // 起步阶段，速度加速度很小，直接使用当前车辆状态作为规划起始点
  if (std::abs(vehicle_state.linear_velocity()) < kEpsilon_v &&
      std::abs(vehicle_state.linear_acceleration()) < kEpsilon_a) {
    reinit_point = ComputeTrajectoryPointFromVehicleState(planning_cycle_time,
                                                          vehicle_state);
  } else {
// 预测车辆位置
// 根据车辆运动学模型，往前推planning_cycle_time时间，预测下一规划周期的状态
    VehicleState predicted_vehicle_state;
    predicted_vehicle_state =
        VehicleModel::Predict(planning_cycle_time, vehicle_state);
// 基于该预测状态生成轨迹点
    reinit_point = ComputeTrajectoryPointFromVehicleState(
        planning_cycle_time, predicted_vehicle_state);
  }
// 数返回值为size为1的数组，即该条拼接轨迹仅有一个轨迹点
  return std::vector<TrajectoryPoint>(1, reinit_point);
}

// only used in navigation mode
void TrajectoryStitcher::TransformLastPublishedTrajectory(
    const double x_diff, const double y_diff, const double theta_diff,
    PublishableTrajectory* prev_trajectory) {
  if (!prev_trajectory) {
    return;
  }

  // R^-1
  double cos_theta = std::cos(theta_diff);
  double sin_theta = -std::sin(theta_diff);

  // -R^-1 * t
  auto tx = -(cos_theta * x_diff - sin_theta * y_diff);
  auto ty = -(sin_theta * x_diff + cos_theta * y_diff);

  std::for_each(prev_trajectory->begin(), prev_trajectory->end(),
                [&cos_theta, &sin_theta, &tx, &ty,
                 &theta_diff](common::TrajectoryPoint& p) {
                  auto x = p.path_point().x();
                  auto y = p.path_point().y();
                  auto theta = p.path_point().theta();

                  auto x_new = cos_theta * x - sin_theta * y + tx;
                  auto y_new = sin_theta * x + cos_theta * y + ty;
                  auto theta_new =
                      common::math::NormalizeAngle(theta - theta_diff);

                  p.mutable_path_point()->set_x(x_new);
                  p.mutable_path_point()->set_y(y_new);
                  p.mutable_path_point()->set_theta(theta_new);
                });
}
/* Planning from current vehicle state if:
   1. the auto-driving mode is off
   (or) 2. we don't have the trajectory from last planning cycle
   (or) 3. the position deviation from actual and target is too high
*/
/// @brief 
/// @param vehicle_chassis 车辆底盘信息
/// @param vehicle_state 车辆状态
/// @param current_timestamp 当前时间戳
/// @param planning_cycle_time 规划周期时间 100ms
/// @param preserved_points_num 保留的轨迹点数目 20
/// @param replan_by_offset 是否根据偏移量重新规划
/// @param prev_trajectory 先前的轨迹
/// @param replan_reason 保存重新规划原因的字符串
/// @return 
std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
    const canbus::Chassis& vehicle_chassis, const VehicleState& vehicle_state,
    const double current_timestamp, const double planning_cycle_time,
    const size_t preserved_points_num, const bool replan_by_offset,
    const PublishableTrajectory* prev_trajectory, std::string* replan_reason) {
  // 如果不使用轨迹拼接
  if (!FLAGS_enable_trajectory_stitcher) {
  // 重规划原因：通过配置文件关闭拼接
    *replan_reason = "stitch is disabled by gflag.";
     // 进入函数，主要利用自行车模型预测未来位置
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
   // 如果没有上一帧轨迹，
  if (!prev_trajectory) {
    *replan_reason = "replan for no previous trajectory.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
// 如果当前驾驶模式不是自动驾驶
  if (vehicle_state.driving_mode() != canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    *replan_reason = "replan for manual mode.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

// 验证先前的轨迹
  size_t prev_trajectory_size = prev_trajectory->NumOfPoints();
// 如果先前轨迹的点数为零（即轨迹为空）
  if (prev_trajectory_size == 0) {
    ADEBUG << "Projected trajectory at time [" << prev_trajectory->header_time()
           << "] size is zero! Previous planning not exist or failed. Use "
              "origin car status instead.";
    *replan_reason = "replan for empty previous trajectory.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  // 第二种方法
// 根据当前帧时间戳以及ADC实际位置信息，在上一帧轨迹寻找匹配点，将上一帧轨迹中匹配点向前看0.1s
// 所对应的轨迹点作为当前帧的规划起始状态点，待当前帧规划生成轨迹后，再和上一帧轨迹中所选择
// 的规划起始点前一段距离的轨迹点进行拼接

// 3.计算时间匹配点：通过 veh_rel_time 查询先前轨迹中与当前时间最接近的轨迹点
// 找到 当前时刻 在上一帧轨迹中的 理论执行位置（按时间推算）
  const double veh_rel_time =
      current_timestamp - prev_trajectory->header_time();
  // 查询先前轨迹中时间最接近的点的索引
  size_t time_matched_index =
      prev_trajectory->QueryLowerBoundPoint(veh_rel_time);

// 4. 检查时间与位置偏差
  // 如果当前时间比先前轨迹的起始时间还要早，则重新规划
  // 检查当前时间是否小于先前轨迹的起始时间
  // 情况1: 当前时间 < 轨迹起始时间（时钟回退 or 轨迹未来）
  if (time_matched_index == 0 &&
      veh_rel_time < prev_trajectory->StartPoint().relative_time()) {
    AWARN << "current time smaller than the previous trajectory's first time";
    *replan_reason =
        "replan for current time smaller than the previous trajectory's first "
        "time.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  // 如果当前时间超出了先前轨迹的时间范围，则重新规划
  // 情况2: 当前时间 > 轨迹结束时间（轨迹已过期）
  if (time_matched_index + 1 >= prev_trajectory_size) {
    AWARN << "current time beyond the previous trajectory's last time";
    *replan_reason =
        "replan for current time beyond the previous trajectory's last time";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  // 获取时间匹配的轨迹点
  auto time_matched_point = prev_trajectory->TrajectoryPointAt(
      static_cast<uint32_t>(time_matched_index));

// 5. 检查路径点
// 如果与当前时间匹配的轨迹点没有路径信息（has_path_point()），则重新规划
  if (!time_matched_point.has_path_point()) {
    *replan_reason = "replan for previous trajectory missed path point";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

// 6.检查位置匹配点
// 查询与车辆当前位置最接近的轨迹点索引
  size_t position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
      {vehicle_state.x(), vehicle_state.y()}, 1.0e-6);

// 计算车辆当前位置在轨迹坐标系中的位置投影（Frenet坐标系）
  auto frenet_sd = ComputePositionProjection(
      vehicle_state.x(), vehicle_state.y(),
      prev_trajectory->TrajectoryPointAt(
          static_cast<uint32_t>(position_matched_index)));
// 如果启用了基于偏移量的重新规划（replan_by_offset 为 true），并且停车制动器已释放（车辆开始移动），则触发重新规划
  if (replan_by_offset) {
    if (vehicle_chassis.has_parking_brake()) {
      static bool parking_brake = true;
      if (parking_brake && !vehicle_chassis.parking_brake()) {
        parking_brake = vehicle_chassis.parking_brake();
        const std::string msg =
            "parking brake off, ego move, replan to avoid large station error";
        AERROR << msg;
        *replan_reason = msg;
        return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                                vehicle_state);
      }
      parking_brake = vehicle_chassis.parking_brake();
    }
// 计算轨迹点与实际位置在纵向（lon_diff）、横向（lat_diff）和时间上的差异
    auto lon_diff = time_matched_point.path_point().s() - frenet_sd.first;
    auto lat_diff = frenet_sd.second;
    double time_diff =
        time_matched_point.relative_time() -
        prev_trajectory
            ->TrajectoryPointAt(static_cast<uint32_t>(position_matched_index))
            .relative_time();

    ADEBUG << "Control lateral diff: " << lat_diff
           << ", longitudinal diff: " << lon_diff
           << ", time diff: " << time_diff;
// 如果横向误差（lat_diff）超过预设阈值，则重新规划
    if (std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold) {  // 0.5
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lat_diff = ",
          lat_diff);
      AERROR << msg;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }
// 如果纵向误差（lon_diff）超过预设阈值，则重新规划
    if (std::fabs(lon_diff) > FLAGS_replan_longitudinal_distance_threshold) {  // 2.5
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lon_diff = ",
          lon_diff);
      AERROR << msg;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }
// 如果时间差异（time_diff）超过预设阈值，则重新规划
    if (std::fabs(time_diff) > FLAGS_replan_time_threshold) {  // 7.0
      const std::string msg = absl::StrCat(
          "the difference between time matched point relative time and "
          "actual position corresponding relative time is too "
          "large. Replan is triggered. time_diff = ",
          time_diff);
      AERROR << msg;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }
  } else {
    ADEBUG << "replan according to certain amount of "
           << "lat、lon and time offset is disabled";
  }


// 如果 所有重规划条件都不满足，进入正常缝合
// 计算前进方向上的相对时间
// 保留到 未来一个规划周期（如当前 10.1s，规划周期 0.1s → 保留到 10.2s）
  double forward_rel_time = veh_rel_time + planning_cycle_time;
// 查找与前进方向时间最接近的轨迹点
  size_t forward_time_index =
      prev_trajectory->QueryLowerBoundPoint(forward_rel_time);
// 输出位置匹配和时间匹配的索引
  ADEBUG << "Position matched index:\t" << position_matched_index;
  ADEBUG << "Time matched index:\t" << time_matched_index;
// 选择时间匹配和位置匹配中较小的索引作为最终的匹配点
// 确定起始索引:取 更保守（靠前） 的匹配点，避免跳过当前位置
  auto matched_index = std::min(time_matched_index, position_matched_index);
// 基于匹配点和保留点数，从先前轨迹中提取出拼接的轨迹部分
// 从 prev_trajectory 中提取一段子轨迹，起始点为 matched_index - preserved_points_num（确保不会越界），结束点为 forward_time_index
// preserved_points_num：额外保留的历史点（如 20 点 ≈ 2 秒）
  std::vector<TrajectoryPoint> stitching_trajectory(
      prev_trajectory->begin() +
          std::max(0, static_cast<int>(matched_index - preserved_points_num)),
      prev_trajectory->begin() + forward_time_index + 1);
  ADEBUG << "stitching_trajectory size: " << stitching_trajectory.size();
// 获取拼接轨迹最后一个点的路径坐标（s值），用于后续归一化
  const double zero_s = stitching_trajectory.back().path_point().s();
// 遍历拼接轨迹，对每个轨迹点进行处理
  for (auto& tp : stitching_trajectory) {
// 如果轨迹点缺少路径点，则重新规划
    if (!tp.has_path_point()) {
      *replan_reason = "replan for previous trajectory missed path point";
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }
    // 调整每个点的相对时间，并将路径坐标归一化
    // 时间归一化：使当前时刻 ≈ 0
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() -
                         current_timestamp);
    // 路程归一化：使当前点 s = 0
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
    // 归一化后，Planner 在 以当前车辆为原点的局部坐标系 中工作
  }
  // 返回拼接后的轨迹
  return stitching_trajectory;
}
/// @brief 
/// @param x 需要投影的目标位置的x坐标
/// @param y 需要投影的目标位置的y坐标
/// @param p 包含一个路径点的信息，这个路径点用于计算目标位置相对于路径的投影
/// @return 
std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(
    const double x, const double y, const TrajectoryPoint& p) {
  // 计算从路径点p到目标点(x,y)的向量v
  Vec2d v(x - p.path_point().x(), y - p.path_point().y());
  // 计算路径方向的单位向量 n，即路径点的切线方向
  Vec2d n(std::cos(p.path_point().theta()), std::sin(p.path_point().theta()));

  std::pair<double, double> frenet_sd;
  // v.InnerProd(n) 计算向量 v 和单位向量 n 的内积（点积）。这表示目标点在路径的切线方向上的投影长度
  frenet_sd.first = v.InnerProd(n) + p.path_point().s();
  // 计算了 v 和 n 的叉积，得到目标点相对于路径的正交坐标 d
  // 对于二维向量，叉积的结果是一个标量，表示向量 v 在垂直于 n 的方向上的分量
  frenet_sd.second = v.CrossProd(n);
  return frenet_sd;
}

}  // namespace planning
}  // namespace apollo
