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

#include "modules/planning/tasks/lane_follow_path/lane_follow_path.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"
namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

bool LaneFollowPath::Init(const std::string& config_dir,
                          const std::string& name,
                          const std::shared_ptr<DependencyInjector>& injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  return Task::LoadConfig<LaneFollowPathConfig>(&config_);
}

/// @brief 
/// @param frame 当前的全局规划帧对象，包含所有障碍物、参考线等信息
/// @param reference_line_info 当前处理的参考线对象，里面包含与该参考线相关的路径、速度、障碍物等数据
/// @return 
apollo::common::Status LaneFollowPath::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  // 已有路径数据 或者 路径可以复用
  if (!reference_line_info->path_data().Empty() ||
      reference_line_info->path_reusable()) {
    ADEBUG << "Skip this time path empty:"
           << reference_line_info->path_data().Empty()
           << "path reusable: " << reference_line_info->path_reusable();
    return Status::OK();
  }
  // 候选路径边界集合
  std::vector<PathBoundary> candidate_path_boundaries;
  // 候选路径数据集合（路径点序列）
  std::vector<PathData> candidate_path_data;
  
  // 获取车辆起点的 SL 坐标（S：沿参考线的距离，L：垂直偏移）
  GetStartPointSLState();

  // 计算候选路径边界（避障的左右限位）
  if (!DecidePathBounds(&candidate_path_boundaries)) {
    AERROR << "Decide path bound failed";
    return Status::OK();
  }
  if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
    AERROR << "Optmize path failed";
    return Status::OK();
  }
  if (!AssessPath(&candidate_path_data,
                  reference_line_info->mutable_path_data())) {
    AERROR << "Path assessment failed";
  }

  return Status::OK();
}
/// @brief 
/// @param boundary 一个指向 std::vector<PathBoundary> 的指针，boundary 用来存储计算出的路径边界
/// @return 
bool LaneFollowPath::DecidePathBounds(std::vector<PathBoundary>* boundary) {
  // 在 boundary 向量的末尾添加一个新的空 PathBoundary
  boundary->emplace_back();
  auto& path_bound = boundary->back();

  // 存储障碍物ID
  std::string blocking_obstacle_id = "";
  // 车道类型
  std::string lane_type = "";
  // 路径最窄宽度
  double path_narrowest_width = 0;

  //  先初始化路径边界为一个非常宽的区域，保证后续有空间收窄
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_,
                                               &path_bound, init_sl_state_)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return false;
  }
  std::string borrow_lane_type;

  // 配置决定是否将自动驾驶车本体边界包含在路径边界内，避免规划路径脱离车辆实际占据空间
  // 将自动驾驶车辆（ADC）包括在车道边界内
  bool is_include_adc = config_.is_extend_lane_bounds_to_include_adc() &&
                        !injector_->planning_context()
                             ->planning_status()
                             .path_decider()
                             .is_in_path_lane_borrow_scenario();
  // 2. Decide a rough boundary based on lane info and ADC's position
  // 根据车道信息和车辆的位置来决定一个粗略的路径边界
  if (!PathBoundsDeciderUtil::GetBoundaryFromSelfLane(
          *reference_line_info_, init_sl_state_, &path_bound)) {
    AERROR << "Failed to decide a rough boundary based on self lane.";
    return false;
  }
 
  // 将车辆占据的空间（含一定缓冲）加入路径边界，避免规划路径侵入车辆自身
  if (is_include_adc) {
    PathBoundsDeciderUtil::ExtendBoundaryByADC(
        *reference_line_info_, init_sl_state_, config_.extend_buffer(),
        &path_bound);
  }
  PrintCurves print_curve;
  
  //  遍历障碍物，将其在路径坐标系上的边界点保存，用于后续可视化或日志调试
  auto indexed_obstacles = reference_line_info_->path_decision()->obstacles();
  for (const auto* obs : indexed_obstacles.Items()) {
    const auto& sl_bound = obs->PerceptionSLBoundary();
    for (int i = 0; i < sl_bound.boundary_point_size(); i++) {
      std::string name = obs->Id() + "_obs_sl_boundary";
      print_curve.AddPoint(
          name, sl_bound.boundary_point(i).s(),
          sl_bound.boundary_point(i).l());
    }
  }
  print_curve.PrintToLog();
  // 3. Fine-tune the boundary based on static obstacles
  PathBound temp_path_bound = path_bound;
  // 根据静态障碍物进一步微调路径边界
  // 会选出最远的那个阻塞障碍物id
  if (!PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
          *reference_line_info_, init_sl_state_, &path_bound,
          &blocking_obstacle_id, &path_narrowest_width)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR << msg;
    return false;
  }
  // 4. Append some extra path bound points to avoid zero-length path data.
  // 如果有阻挡障碍物，给路径边界尾部加额外点，防止路径变成零长度
  // 向路径边界中添加一些额外的边界点，以避免生成零长度的路径数据
  int counter = 0;
  while (!blocking_obstacle_id.empty() &&
         path_bound.size() < temp_path_bound.size() &&
         counter < FLAGS_num_extra_tail_bound_point) {  // 20
    path_bound.push_back(temp_path_bound[path_bound.size()]);
    counter++;
  }

  //  更新规划状态中的障碍物阻塞信息
  // lane_follow_status update
  auto* lane_follow_status = injector_->planning_context()
                                    ->mutable_planning_status()
                                    ->mutable_lane_follow();
  if (!blocking_obstacle_id.empty()) {
    // 如果有阻塞的障碍物，记录阻塞的障碍物ID，并更新阻塞持续时间
    double current_time = ::apollo::cyber::Clock::NowInSeconds();
    lane_follow_status->set_block_obstacle_id(blocking_obstacle_id);
    if (lane_follow_status->lane_follow_block()) {
      lane_follow_status->set_block_duration(
          lane_follow_status->block_duration() + current_time -
          lane_follow_status->last_block_timestamp());
    } else {
      lane_follow_status->set_block_duration(0);
      lane_follow_status->set_lane_follow_block(true);
    }
    lane_follow_status->set_last_block_timestamp(current_time);
  } else { 
    // 无阻塞，重置相关状态
    if (lane_follow_status->lane_follow_block()) {
      lane_follow_status->set_block_duration(0);
      lane_follow_status->set_lane_follow_block(false);
      lane_follow_status->set_last_block_timestamp(0);
    }
  }

  ADEBUG << "Completed generating path boundaries.";
  // 检查初始车辆横向位置是否在路径边界内
  // 如果需要包括 ADC，检查初始化时车辆的横向位置是否在路径边界内。如果不在边界内，表示可能发生了车道借道，返回 false
  if (is_include_adc) {
    CHECK_LE(init_sl_state_.second[0], path_bound[0].l_upper.l);
    CHECK_GE(init_sl_state_.second[0], path_bound[0].l_lower.l);
  }
  if (init_sl_state_.second[0] > path_bound[0].l_upper.l ||
      init_sl_state_.second[0] < path_bound[0].l_lower.l) {
    AINFO << "not in self lane maybe lane borrow , init l : "
          << init_sl_state_.second[0] << ", path_bound l: [ "
          << path_bound[0].l_lower.l << ","
          << path_bound[0].l_upper.l << " ]";
    return false;
  }
  // std::vector<std::pair<double, double>> regular_path_bound_pair;
  // for (size_t i = 0; i < path_bound.size(); ++i) {
  //   regular_path_bound_pair.emplace_back(std::get<1>(path_bound[i]),
  //                                        std::get<2>(path_bound[i]));
  // }
  // 设置路径边界的标签，并记录调试信息（包括边界标签和阻塞障碍物ID）
  path_bound.set_label(absl::StrCat("regular/", "self"));
  path_bound.set_blocking_obstacle_id(blocking_obstacle_id);
  RecordDebugInfo(path_bound, path_bound.label(), reference_line_info_);
  return true;
}

/// @brief 
/// @param path_boundaries 
/// @param candidate_path_data 
/// @return 
bool LaneFollowPath::OptimizePath(
    const std::vector<PathBoundary>& path_boundaries,
    std::vector<PathData>* candidate_path_data) {
  // 从成员变量中提取路径优化器的参数配置
  const auto& config = config_.path_optimizer_config();
  // 获取参考线: 用于计算曲率（kappa）、起止点坐标等几何信息
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  // 设置终点状态: 路径优化器使用起点、终点状态作为优化约束，这里设定终点为 0 横向偏移、0 速度、0 加速度
  std::array<double, 3> end_state = {0.0, 0.0, 0.0};
  // 遍历所有路径边界: 对每一条候选路径边界执行一次路径优化
  for (const auto& path_boundary : path_boundaries) {
    // 路径边界校验: 确保路径边界的点数量足够（至少两个点），否则报错返回失败
    size_t path_boundary_size = path_boundary.boundary().size();
    if (path_boundary_size <= 1U) {
      AERROR << "Get invalid path boundary with size: " << path_boundary_size;
      return false;
    }
    // 定义优化结果容器: 存储优化得到的横向偏移 l、一阶导数 dl（速度）和二阶导数 ddl（曲率变化）
    std::vector<double> opt_l, opt_dl, opt_ddl;
    // 计算曲率二阶导（加速度）的边界：根据路径边界和参考线信息，计算每个点允许的最大曲率变化范围（上下限）
    std::vector<std::pair<double, double>> ddl_bounds;
    PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line,
                                         &ddl_bounds);
    // 调试输出参考线曲率：收集每个点对应的参考线曲率，并输出至日志用于调试（可视化工具查看路径曲率变化）
    PrintCurves print_debug;
    for (size_t i = 0; i < path_boundary_size; ++i) {
      double s = static_cast<double>(i) * path_boundary.delta_s() +
                 path_boundary.start_s();
      double kappa = reference_line.GetNearestReferencePoint(s).kappa();
      print_debug.AddPoint(
          "ref_kappa", static_cast<double>(i) * path_boundary.delta_s(), kappa);
    }
    print_debug.PrintToLog();
    
    // 估计路径平滑性限制（jerk 限制）：通过当前初始状态的横向速度估算路径允许的最大三阶导数（jerk）边界
    const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(
        std::fmax(init_sl_state_.first[1], 1e-12));
    // 生成路径参考偏移（ref_l）及其权重：通常以中心线或安全区域为参考，引导优化结果向安全区域居中，同时赋予不同位置不同的优化权重
    std::vector<double> ref_l(path_boundary_size, 0);
    std::vector<double> weight_ref_l(path_boundary_size, 0);
    PathOptimizerUtil::UpdatePathRefWithBound(
        path_boundary, config.path_reference_l_weight(), &ref_l, &weight_ref_l);
    // 执行路径优化： 对当前 path_boundary 区域执行最小化 cost 的轨迹规划，结果保存在 opt_l, opt_dl, opt_ddl 中
    bool res_opt = PathOptimizerUtil::OptimizePath(
        init_sl_state_, end_state, ref_l, weight_ref_l, path_boundary,
        ddl_bounds, jerk_bound, config, &opt_l, &opt_dl, &opt_ddl);
    if (res_opt) {
      // 生成 Frenet 坐标系下的路径结构： 将优化结果转为可用于后续坐标转换与跟踪控制的结构体
      auto frenet_frame_path = PathOptimizerUtil::ToPiecewiseJerkPath(
          opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
          path_boundary.start_s());
      //  构造路径数据对象 PathData：绑定参考线并设置刚才优化得到的 Frenet 路径
      PathData path_data;
      path_data.SetReferenceLine(&reference_line);
      path_data.SetFrenetPath(std::move(frenet_frame_path));
      // 前轴 / 后轴参考点转换：某些规划场景下需要以前轴中心（或后轴中心）为参考点进行路径转换，以匹配控制器模型
      if (FLAGS_use_front_axe_center_in_path_planning) {
        auto discretized_path = DiscretizedPath(
            PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(
                path_data));
        path_data.SetDiscretizedPath(discretized_path);
      }
      // 设置路径标签和阻塞障碍物 ID
      // path_label：用于标识该路径用途（如主道、换道、临停等）
      // blocking_obstacle_id：路径被谁阻挡了，便于后续决策使用
      path_data.set_path_label(path_boundary.label());
      path_data.set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
      candidate_path_data->push_back(std::move(path_data));
    }
  }
  if (candidate_path_data->empty()) {
    return false;
  }
  return true;
}

bool LaneFollowPath::AssessPath(std::vector<PathData>* candidate_path_data,
                                PathData* final_path) {
  PathData& curr_path_data = candidate_path_data->back();
  RecordDebugInfo(curr_path_data, curr_path_data.path_label(),
                  reference_line_info_);
  if (!PathAssessmentDeciderUtil::IsValidRegularPath(*reference_line_info_,
                                                     curr_path_data)) {
    AINFO << "Lane follow path is invalid";
    return false;
  }

  std::vector<PathPointDecision> path_decision;
  PathAssessmentDeciderUtil::InitPathPointDecision(
      curr_path_data, PathData::PathPointType::IN_LANE, &path_decision);
  curr_path_data.SetPathPointDecisionGuide(std::move(path_decision));

  if (curr_path_data.Empty()) {
    AINFO << "Lane follow path is empty after trimed";
    return false;
  }
  *final_path = curr_path_data;
  AINFO << final_path->path_label() << final_path->blocking_obstacle_id();
  reference_line_info_->MutableCandidatePathData()->push_back(*final_path);
  reference_line_info_->SetBlockingObstacle(
      curr_path_data.blocking_obstacle_id());
  return true;
}

}  // namespace planning
}  // namespace apollo
