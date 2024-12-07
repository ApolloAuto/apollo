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

#include "modules/planning/tasks/speed_bounds_decider/st_boundary_mapper.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common_msgs/planning_msgs/decision.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::common::math::Polygon2d;

STBoundaryMapper::STBoundaryMapper(
    const SpeedBoundsDeciderConfig& config, const ReferenceLine& reference_line,
    const PathData& path_data, const double planning_distance,
    const double planning_time,
    const std::shared_ptr<DependencyInjector>& injector)
    : speed_bounds_config_(config),
      reference_line_(reference_line),
      path_data_(path_data),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()),
      planning_max_distance_(planning_distance),
      planning_max_time_(planning_time),
      injector_(injector) {}
/// @brief 
/// @param path_decision 一个 PathDecision* 类型的参数 path_decision，表示路径决策对象，包含路径规划过程中所有的障碍物和决策信息
/// @return 
Status STBoundaryMapper::ComputeSTBoundary(PathDecision* path_decision) const {
  // Sanity checks.
  // 确保 planning_max_time_（规划最大时间）大于 0，确保时间配置有效
  CHECK_GT(planning_max_time_, 0.0);
  // 检查路径点的数量是否少于 2。如果路径点数量不足，输出错误信息并返回 PLANNING_ERROR 状态
  if (path_data_.discretized_path().size() < 2) {
    AERROR << "Fail to get params because of too few path points. path points "
              "size: "
           << path_data_.discretized_path().size() << ".";
    return Status(ErrorCode::PLANNING_ERROR,
                  "Fail to get params because of too few path points");
  }

  // Go through every obstacle.
  // 记录最近的停车障碍物
  Obstacle* stop_obstacle = nullptr;   // 后面需要更新的
  // 记录停车决策
  ObjectDecisionType stop_decision;    //  后面需要更新的
  // 用于存储最小停车点的 s 值，初始化为最大值
  double min_stop_s = std::numeric_limits<double>::max();   // 后面需要更新的
  // 遍历路径决策中的每个障碍物（path_decision->obstacles().Items() 返回所有障碍物），并通过 Find 方法获取障碍物对象 ptr_obstacle
  for (const auto* ptr_obstacle_item : path_decision->obstacles().Items()) {
    Obstacle* ptr_obstacle = path_decision->Find(ptr_obstacle_item->Id());
    ACHECK(ptr_obstacle != nullptr);

    // If no longitudinal decision has been made, then plot it onto ST-graph.
    // 如果该障碍物没有纵向决策（即没有决定是否停车、是否超车等），则将其时空边界绘制到 ST 图中，调用 ComputeSTBoundary 递归处理该障碍物，然后继续处理下一个障碍物
    if (!ptr_obstacle->HasLongitudinalDecision()) {
      ComputeSTBoundary(ptr_obstacle);
      continue;
    }

    // If there is a longitudinal decision, then fine-tune boundary.
    // 如果障碍物有纵向决策，则获取其决策，存储在 decision 中。该决策可能包括停车、跟随、超车、让行等
    const auto& decision = ptr_obstacle->LongitudinalDecision();
    // 如果决策包含停车 (has_stop())
    if (decision.has_stop()) {
      // 1. Store the closest stop fence info.
      // TODO(all): store ref. s value in stop decision; refine the code then.
      // 将停车点信息存储起来。通过 XYToSL 方法将停车点从 XY 坐标转换为参考线上的 s 和 l 坐标，存储在 stop_sl_point 中
      common::SLPoint stop_sl_point;
      reference_line_.XYToSL(decision.stop().stop_point(), &stop_sl_point);
      // 获取停车点的 s 值，存储为 stop_s
      const double stop_s = stop_sl_point.s();
      // 如果当前障碍物的停车点 stop_s 小于记录的最小停车点 min_stop_s
      if (stop_s < min_stop_s) {
        stop_obstacle = ptr_obstacle;
        min_stop_s = stop_s;
        stop_decision = decision;
      }
    } else if (decision.has_follow() || decision.has_overtake() ||
               decision.has_yield()) {
      // 2. Depending on the longitudinal overtake/yield decision,
      //    fine-tune the upper/lower st-boundary of related obstacles.
      // 如果决策包含跟随、超车或让行（has_follow()、has_overtake()、has_yield()），则根据这些决策调整时空边界。
      ComputeSTBoundaryWithDecision(ptr_obstacle, decision);
    } else if (!decision.has_ignore()) {
      // 3. Ignore those unrelated obstacles.
      // 如果决策是 ignore（忽略）以外的其他类型，则记录警告，说明当前决策没有被处理或没有映射
      AWARN << "No mapping for decision: " << decision.DebugString();
    }
  }
  // 如果找到停车决策（stop_obstacle 非空），则调用 MapStopDecision 函数来处理停车决策并更新相应的时空边界
  if (stop_obstacle) {
    bool success = MapStopDecision(stop_obstacle, stop_decision);
    if (!success) {
      const std::string msg = "Fail to MapStopDecision.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  return Status::OK();
}

bool STBoundaryMapper::MapStopDecision(
    Obstacle* stop_obstacle, const ObjectDecisionType& stop_decision) const {
  DCHECK(stop_decision.has_stop()) << "Must have stop decision";
  common::SLPoint stop_sl_point;
  reference_line_.XYToSL(stop_decision.stop().stop_point(), &stop_sl_point);

  double st_stop_s = 0.0;
  const double stop_ref_s =
      stop_sl_point.s() - vehicle_param_.front_edge_to_center();

  if (stop_ref_s > path_data_.frenet_frame_path().back().s()) {
    st_stop_s = path_data_.discretized_path().back().s() +
                (stop_ref_s - path_data_.frenet_frame_path().back().s());
  } else {
    PathPoint stop_point;
    if (!path_data_.GetPathPointWithRefS(stop_ref_s, &stop_point)) {
      return false;
    }
    st_stop_s = stop_point.s();
  }

  const double s_min = std::fmax(0.0, st_stop_s);
  const double s_max = std::fmax(
      s_min, std::fmax(planning_max_distance_, reference_line_.Length()));

  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  point_pairs.emplace_back(STPoint(s_min, 0.0), STPoint(s_max, 0.0));
  point_pairs.emplace_back(
      STPoint(s_min, planning_max_time_),
      STPoint(s_max + speed_bounds_config_.boundary_buffer(),
              planning_max_time_));
  auto boundary = STBoundary(point_pairs);
  boundary.SetBoundaryType(STBoundary::BoundaryType::STOP);
  boundary.SetCharacteristicLength(speed_bounds_config_.boundary_buffer());
  boundary.set_id(stop_obstacle->Id());
  stop_obstacle->set_path_st_boundary(boundary);
  return true;
}
/// @brief 计算并更新与某个障碍物（obstacle）相关的时间-空间（ST）边界
/// @param obstacle 
void STBoundaryMapper::ComputeSTBoundary(Obstacle* obstacle) const {
  if (FLAGS_use_st_drivable_boundary) {   // 默认为false
    return;
  }
  // 存储计算出的时间-空间边界的下边界点
  std::vector<STPoint> lower_points;
  // 存储计算出的时间-空间边界的上边界点
  std::vector<STPoint> upper_points;
  // 计算出重叠的边界点
  if (!GetOverlapBoundaryPoints(path_data_.discretized_path(), *obstacle,
                                &upper_points, &lower_points)) {
    return;
  }
  // 使用之前计算得到的上下边界点创建一个 STBoundary 对象 boundary
  auto boundary = STBoundary::CreateInstance(lower_points, upper_points);
  // 将该障碍物的 ID 设置给 boundary 对象，确保边界与特定的障碍物关联
  boundary.set_id(obstacle->Id());

  // TODO(all): potential bug here.
  const auto& prev_st_boundary = obstacle->path_st_boundary();
  const auto& ref_line_st_boundary = obstacle->reference_line_st_boundary();
  if (!prev_st_boundary.IsEmpty()) {
    boundary.SetBoundaryType(prev_st_boundary.boundary_type());
  } else if (!ref_line_st_boundary.IsEmpty()) {
    boundary.SetBoundaryType(ref_line_st_boundary.boundary_type());
  }
 // 将计算得到的 boundary 设置为障碍物的路径 ST 边界，更新障碍物的状态
  obstacle->set_path_st_boundary(boundary);
}
/// @brief 根据路径点和障碍物的轨迹计算可能的重叠区域，并返回这些区域的上下边界点
/// @param path_points 表示路径上的离散点，是一个 PathPoint 类型的向量
/// @param obstacle 表示障碍物对象，用于获取障碍物的信息，如轨迹、尺寸等
/// @param upper_points 保存重叠区域的上边界
/// @param lower_points 保存重叠区域的下边界
/// @return 返回一个 bool 类型，表示是否找到了有效的重叠边界点
bool STBoundaryMapper::GetOverlapBoundaryPoints(
    const std::vector<PathPoint>& path_points, const Obstacle& obstacle,
    std::vector<STPoint>* upper_points,
    std::vector<STPoint>* lower_points) const {
  // Sanity checks.
  // 数据有效性检查
  DCHECK(upper_points->empty());
  DCHECK(lower_points->empty());
  if (path_points.empty()) {
    AERROR << "No points in path_data_.discretized_path().";
    return false;
  }

  const auto* planning_status = injector_->planning_context()
                                    ->mutable_planning_status()
                                    ->mutable_change_lane();
  // 根据是否处于换道，调整l_buffer
  double l_buffer =
      planning_status->status() == ChangeLaneStatus::IN_CHANGE_LANE
          ? speed_bounds_config_.lane_change_obstacle_nudge_l_buffer()    // 0.3
          : FLAGS_nonstatic_obstacle_nudge_l_buffer;   // 0.4

  // Draw the given obstacle on the ST-graph.
  // 获取障碍物的轨迹
  const auto& trajectory = obstacle.Trajectory();
  // 获取障碍物的长度
  const double obstacle_length = obstacle.Perception().length();
  // 获取障碍物的宽度
  const double obstacle_width = obstacle.Perception().width();
  if (trajectory.trajectory_point().empty()) {
    // 处理静态障碍物
    // 判断是否有重叠
    bool box_check_collision = false;

    // For those with no predicted trajectories, just map the obstacle's
    // current position to ST-graph and always assume it's static.
    // 如果障碍物不是静态的，但没有预测轨迹，打印警告信息
    if (!obstacle.IsStatic()) {
      AWARN << "Non-static obstacle[" << obstacle.Id()
            << "] has NO prediction trajectory."
            << obstacle.Perception().ShortDebugString();
    }
    // 获取障碍物的包围盒（Box2d），用于碰撞检测
    const Box2d& obs_box = obstacle.PerceptionBoundingBox();   
  // 遍历路径上的每个点，检查每个路径点是否与障碍物的包围盒发生重叠
    for (const auto& curr_point_on_path : path_points) { 
      // 遍历离散路径点，若当前点超过了规划最大距离，退出
      if (curr_point_on_path.s() > planning_max_distance_) {
        break;
      }
      // 如果发生重叠，设置 box_check_collision 为 true 并跳出循环
      if (CheckOverlap(curr_point_on_path, obs_box, l_buffer)) {
        box_check_collision = true;
        break;
      }
    }

    if (box_check_collision) {
      const double backward_distance = -vehicle_param_.front_edge_to_center();
      const double forward_distance = obs_box.length();
      // 遍历路径点，检查是否与障碍物的多边形（Polygon2d）发生重叠
      for (const auto& curr_point_on_path : path_points) {
        if (curr_point_on_path.s() > planning_max_distance_) {
          break;
        }
        const Polygon2d& obs_polygon = obstacle.PerceptionPolygon();
        if (CheckOverlap(curr_point_on_path, obs_polygon, l_buffer)) {
          // If there is overlapping, then plot it on ST-graph.
          // 计算重叠区域的 s 值（路径上的位置）。low_s 为重叠区的开始位置，high_s 为重叠区的结束位置
          double low_s =
              std::fmax(0.0, curr_point_on_path.s() + backward_distance);
          double high_s = std::fmin(planning_max_distance_,
                                    curr_point_on_path.s() + forward_distance);
          // It is an unrotated rectangle appearing on the ST-graph.
          // TODO(jiacheng): reconsider the backward_distance, it might be
          // unnecessary, but forward_distance is indeed meaningful though.
          // 将重叠区域的上下边界点添加到 upper_points 和 lower_points 中，并退出循环
          lower_points->emplace_back(low_s, 0.0);
          lower_points->emplace_back(low_s, planning_max_time_);
          upper_points->emplace_back(high_s, 0.0);
          upper_points->emplace_back(high_s, planning_max_time_);
          break;
        }
      }
    }
  } else {
    // 处理动态障碍物
    // For those with predicted trajectories (moving obstacles):
    // 1. Subsample to reduce computation time.
    // 如果障碍物有预测轨迹（动态障碍物），则对路径点进行采样，减少计算量
    // 若路径点数目过多，降采样
    const int default_num_point = 50;
    DiscretizedPath discretized_path;
    if (path_points.size() > 2 * default_num_point) {
      const auto ratio = path_points.size() / default_num_point;
      std::vector<PathPoint> sampled_path_points;
      for (size_t i = 0; i < path_points.size(); ++i) {
        if (i % ratio == 0) {
          sampled_path_points.push_back(path_points[i]);
        }
      }
      discretized_path = DiscretizedPath(std::move(sampled_path_points));
    } else {
      discretized_path = DiscretizedPath(path_points);
    }

    // 2. Go through every point of the predicted obstacle trajectory.
    // 根据障碍物的速度和路径点间隔，计算用于轨迹点的步长（trajectory_step），即每次检查的轨迹点数量
    double trajectory_time_interval =
              obstacle.Trajectory().trajectory_point()[1].relative_time();
    int trajectory_step = std::min(
                            FLAGS_trajectory_check_collision_time_step,
                            std::max(vehicle_param_.width() / obstacle.speed()
                          / trajectory_time_interval, 1.0));
   // 初始化轨迹点的碰撞状态
    bool trajectory_point_collision_status = false;
    int previous_index = 0;
   // 遍历障碍物的每个轨迹点，并计算其形状（多边形）
    for (int i = 0; i < trajectory.trajectory_point_size();
          i = std::min(i + trajectory_step,
                        trajectory.trajectory_point_size() - 1)) {
      const auto& trajectory_point = trajectory.trajectory_point(i);
      Polygon2d obstacle_shape =
                  obstacle.GetObstacleTrajectoryPolygon(trajectory_point);
    // 跳过负时间的轨迹点
      double trajectory_point_time = trajectory_point.relative_time();
      static constexpr double kNegtiveTimeThreshold = -1.0;
      // 跳过小于阈值时间的轨迹点
      if (trajectory_point_time < kNegtiveTimeThreshold) {
        continue;
      }
      // 检查路径点是否与当前轨迹点的障碍物形状发生重叠
      bool collision = CheckOverlapWithTrajectoryPoint(
                                      discretized_path, obstacle_shape,
                                      upper_points, lower_points,
                                      l_buffer, default_num_point,
                                      obstacle_length, obstacle_width,
                                      trajectory_point_time);
     // 如果碰撞状态发生变化，则回溯检查之前的轨迹点
      if ((trajectory_point_collision_status ^ collision) && i != 0) {
        // Start retracing track points forward
        int index = i - 1;
        while ((trajectory_point_collision_status ^ collision)
                  && index > previous_index) {
                    // 回溯并更新碰撞状态
          const auto& point = trajectory.trajectory_point(index);
          trajectory_point_time = point.relative_time();
          obstacle_shape = obstacle.GetObstacleTrajectoryPolygon(point);
          collision = CheckOverlapWithTrajectoryPoint(
                                      discretized_path, obstacle_shape,
                                      upper_points, lower_points,
                                      l_buffer, default_num_point,
                                      obstacle_length, obstacle_width,
                                      trajectory_point_time);
          index--;
        }
        trajectory_point_collision_status = !trajectory_point_collision_status;
      }
      // 如果到达最后一个轨迹点，退出循环
      if (i == trajectory.trajectory_point_size() - 1) break;
      previous_index = i;
    }
  }

  // Sanity checks and return.
  std::sort(lower_points->begin(), lower_points->end(),
            [](const STPoint& a, const STPoint& b) {
              return a.t() < b.t();
            });
  std::sort(upper_points->begin(), upper_points->end(),
            [](const STPoint& a, const STPoint& b) {
              return a.t() < b.t();
            });
  DCHECK_EQ(lower_points->size(), upper_points->size());
  return (lower_points->size() > 1 && upper_points->size() > 1);
}
/// @brief 检查给定的轨迹点是否与障碍物发生重叠。如果发生重叠，函数将使用更高的分辨率定位与障碍物重叠的具体区间，并返回重叠区域的上下边界
/// @param discretized_path 轨迹的离散化路径，包含了路径的多个点，每个点都有位置和时间信息
/// @param obstacle_shape 障碍物的多边形形状，用于与轨迹点进行碰撞检测
/// @param upper_points 重叠区域的上边界
/// @param lower_points 重叠区域的下边界
/// @param l_buffer 一个缓冲区，通常用于防止因计算误差导致的假重叠
/// @param default_num_point 默认的路径分割点数，用于更精细的分辨率
/// @param obstacle_length 障碍物的长度，用于估计重叠区域的大小
/// @param obstacle_width 障碍物的宽度，用于估计重叠区域的大小
/// @param trajectory_point_time 轨迹点的时间，表示车辆经过该点时的时间
/// @return 
bool STBoundaryMapper::CheckOverlapWithTrajectoryPoint(
    const DiscretizedPath& discretized_path,
    const Polygon2d& obstacle_shape,
    std::vector<STPoint>* upper_points,
    std::vector<STPoint>* lower_points,
    const double l_buffer,
    int default_num_point,
    const double obstacle_length,
    const double obstacle_width,
    const double trajectory_point_time) const {
  // 车辆前端到车辆中心的距离，用于控制路径点的间隔。每次检查的间隔为 step_length
  const double step_length = vehicle_param_.front_edge_to_center();
  // 路径的长度，限制在最大轨迹长度（speed_bounds_config_.max_trajectory_len()）和轨迹实际长度之间的较小值
  auto path_len = std::min(speed_bounds_config_.max_trajectory_len(),
                               discretized_path.Length());
  // Go through every point of the ADC's path.
  for (double path_s = 0.0; path_s < path_len; path_s += step_length) {
    const auto curr_adc_path_point =
        discretized_path.Evaluate(path_s + discretized_path.front().s());
    if (CheckOverlap(curr_adc_path_point, obstacle_shape, l_buffer)) {
      // Found overlap, start searching with higher resolution
      // 高分辨率查找重叠区间
      const double backward_distance = -step_length;
      const double forward_distance = vehicle_param_.length() +
                                      vehicle_param_.width() +
                                      obstacle_length + obstacle_width;
      const double default_min_step = 0.1;  // in meters
      const double fine_tuning_step_length = std::fmin(
          default_min_step, discretized_path.Length() / default_num_point);
      // 双向细化搜索
      bool find_low = false;
      bool find_high = false;
      double low_s = std::fmax(0.0, path_s + backward_distance); // 向后搜索
      double high_s =
          std::fmin(discretized_path.Length(), path_s + forward_distance);  // 向前搜索

      // Keep shrinking by the resolution bidirectionally until finally
      // locating the tight upper and lower bounds.
      while (low_s < high_s) {
        if (find_low && find_high) {
          break;
        }
        if (!find_low) {
          const auto& point_low = discretized_path.Evaluate(
              low_s + discretized_path.front().s());
          if (!CheckOverlap(point_low, obstacle_shape, l_buffer)) {
            low_s += fine_tuning_step_length;
          } else {
            find_low = true;
          }
        }
        if (!find_high) {
          const auto& point_high = discretized_path.Evaluate(
              high_s + discretized_path.front().s());
          if (!CheckOverlap(point_high, obstacle_shape, l_buffer)) {
            high_s -= fine_tuning_step_length;
          } else {
            find_high = true;
          }
        }
      }
      // 保存重叠区间
      if (find_high && find_low) {
        lower_points->emplace_back(
            low_s - speed_bounds_config_.point_extension(),
            trajectory_point_time);
        upper_points->emplace_back(
            high_s + speed_bounds_config_.point_extension(),
            trajectory_point_time);
      }
      return true;
    }
  }
  return false;
}
/// @brief 它用于根据给定的障碍物（obstacle）和决策（decision）来计算ST（时空）边界
/// @param obstacle 一个指向 Obstacle 对象的指针，表示障碍物
/// @param decision 一个 ObjectDecisionType 类型的常量引用，表示决策
void STBoundaryMapper::ComputeSTBoundaryWithDecision(
    Obstacle* obstacle, const ObjectDecisionType& decision) const {
// 使用 DCHECK 检查 decision 必须包含 follow、yield 或 overtake 之一，表示决策的类型
// 如果决策不符合要求，则打印出调试信息
  DCHECK(decision.has_follow() || decision.has_yield() ||
         decision.has_overtake())
      << "decision is " << decision.DebugString()
      << ", but it must be follow or yield or overtake.";
// 存储时空边界的下界点
  std::vector<STPoint> lower_points;
// 存储时空边界的上界点
  std::vector<STPoint> upper_points;

  if (FLAGS_use_st_drivable_boundary &&
      obstacle->is_path_st_boundary_initialized()) {
    const auto& path_st_boundary = obstacle->path_st_boundary();
    lower_points = path_st_boundary.lower_points();
    upper_points = path_st_boundary.upper_points();
  } else {
    if (!GetOverlapBoundaryPoints(path_data_.discretized_path(), *obstacle,
                                  &upper_points, &lower_points)) {
      return;
    }
  }
 // 使用 lower_points 和 upper_points 创建一个 STBoundary 实例，表示时空边界
  auto boundary = STBoundary::CreateInstance(lower_points, upper_points);

  // get characteristic_length and boundary_type.
  // 初始化边界类型 b_type 为 UNKNOWN
  STBoundary::BoundaryType b_type = STBoundary::BoundaryType::UNKNOWN;
  // 初始化 characteristic_length（特征长度）为 0
  double characteristic_length = 0.0;
  if (decision.has_follow()) {
    characteristic_length = std::fabs(decision.follow().distance_s());  // 这个距离怎么来的
    AINFO << "characteristic_length: " << characteristic_length;
    // 扩展边界
    boundary = STBoundary::CreateInstance(lower_points, upper_points)
                   .ExpandByS(characteristic_length);
    b_type = STBoundary::BoundaryType::FOLLOW;
  } else if (decision.has_yield()) {
    characteristic_length = std::fabs(decision.yield().distance_s());
    boundary = STBoundary::CreateInstance(lower_points, upper_points)
                   .ExpandByS(characteristic_length);
    b_type = STBoundary::BoundaryType::YIELD;
  } else if (decision.has_overtake()) {
    characteristic_length = std::fabs(decision.overtake().distance_s());
    b_type = STBoundary::BoundaryType::OVERTAKE;
  } else {
    // 如果决策既不是 follow、yield 也不是 overtake，则触发断言错误，打印出决策的调试信息
    DCHECK(false) << "Obj decision should be either yield or overtake: "
                  << decision.DebugString();
  }
  // 设置边界的类型为之前计算出的 b_type
  boundary.SetBoundaryType(b_type);
  // 设置边界的 ID 为障碍物的 ID
  boundary.set_id(obstacle->Id());
  // 设置边界的特征长度
  boundary.SetCharacteristicLength(characteristic_length);
  // 将计算得到的时空边界设置回障碍物中
  obstacle->set_path_st_boundary(boundary);
}
/// @brief 检查两个矩形框（一个表示障碍物，另一个表示自动驾驶车辆的边界框）是否有重叠
/// @param path_point 当前路径点，包含了车辆的位置 (x, y) 和朝向角度 theta
/// @param obs_box 障碍物的边界框，类型为 Box2d
/// @param l_buffer 一个缓冲区长度，用来扩展车辆的边界框
/// @return 返回一个布尔值，表示两者是否重叠
bool STBoundaryMapper::CheckOverlap(const PathPoint& path_point,
                                    const Box2d& obs_box,
                                    const double l_buffer) const {
  // Convert reference point from center of rear axis to center of ADC.
  // 车辆几何中心在车体坐标系下的坐标
  Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                              vehicle_param_.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param_.left_edge_to_center() -
                              vehicle_param_.right_edge_to_center()) *
                                 0.5);
  // 将后轴中心变换到几何中心
  ego_center_map_frame.SelfRotate(path_point.theta());
  ego_center_map_frame.set_x(ego_center_map_frame.x() + path_point.x());
  ego_center_map_frame.set_y(ego_center_map_frame.y() + path_point.y());

  // Compute the ADC bounding box.
  // 计算车辆的boundingbox
  Box2d adc_box(ego_center_map_frame, path_point.theta(),
                vehicle_param_.length(), vehicle_param_.width() + l_buffer * 2);

  // Check whether ADC bounding box overlaps with obstacle bounding box.
  // 碰撞检测
  return obs_box.HasOverlap(adc_box);
}
/// @brief 首先通过路径点计算出自车在地图上的位置和朝向，然后根据自车的尺寸（包括缓冲区）计算出自车的包围盒，
//并将其转化为多边形。最后，通过与障碍物多边形的重叠检测来判断是否存在碰撞或障碍
/// @param path_point 路径点，表示自车当前位置及其朝向（theta）
/// @param obs_polygon 障碍物多边形，表示障碍物的位置和形状
/// @param l_buffer 一个额外的缓冲区长度，用来扩展自车的宽度
/// @return 
bool STBoundaryMapper::CheckOverlap(const PathPoint& path_point,
                                    const Polygon2d& obs_polygon,
                                    const double l_buffer) const {
  // Convert reference point from center of rear axis to center of ADC.
  // 目的是将参考点从自车后轴中心转换为自车的中心
  // 计算自车中心在车辆坐标系中的位置（ego_center_map_frame）
  Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                              vehicle_param_.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param_.left_edge_to_center() -
                              vehicle_param_.right_edge_to_center()) *
                                 0.5);
  ego_center_map_frame.SelfRotate(path_point.theta());
  ego_center_map_frame.set_x(ego_center_map_frame.x() + path_point.x());
  ego_center_map_frame.set_y(ego_center_map_frame.y() + path_point.y());

  // Compute the ADC bounding box.
  Box2d adc_box(ego_center_map_frame, path_point.theta(),
                vehicle_param_.length(), vehicle_param_.width() + l_buffer * 2);

  // Check whether ADC polygon overlaps with obstacle polygon.
  // 检查自车的多边形是否与障碍物的多边形重叠
  // 将自车的包围盒（adc_box）转化为一个多边形（adc_polygon），该多边形表示自车的形状
  Polygon2d adc_polygon(adc_box);
  return obs_polygon.HasOverlap(adc_polygon);
}

}  // namespace planning
}  // namespace apollo
