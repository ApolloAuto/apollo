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
// 用于 std::max 等算法
#include <algorithm>
// 智能指针（如 shared_ptr）
#include <memory>
// 字符串操作
#include <string>
// std::pair, std::move
#include <utility>
// 动态数组
#include <vector>
// 获取车辆几何参数（如轴距）
#include "modules/common/configs/vehicle_config_helper.h"
// 调试绘图工具（PrintCurves）
#include "modules/planning/planning_base/common/util/print_debug_info.h"
// 路径生成通用接口
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
// 路径评估
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"
// 路径边界生成
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
// 路径优化
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"
namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

bool LaneFollowPath::Init(const std::string& config_dir,
                          const std::string& name,
                          const std::shared_ptr<DependencyInjector>& injector) {
  // 调用父类 Task::Init() 完成基础初始化
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  // 从配置文件加载 LaneFollowPathConfig 到成员变量 config_
  // 使用模板方法 LoadConfig<T>() 自动解析 protobuf 配置。
  return Task::LoadConfig<LaneFollowPathConfig>(&config_);
}

/// @brief 
/// @param frame 当前规划周期的全局状态（含自车状态、地图、障碍物等）
/// @param reference_line_info 当前参考线相关信息（含路径、决策、障碍物等）
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
  // 候选路径边界（横向可行区域）
  std::vector<PathBoundary> candidate_path_boundaries;
  // 候选路径（Frenet 坐标下的 L(s) 曲线）
  std::vector<PathData> candidate_path_data;
  
  // 获取车辆起点的 SL 坐标（S：沿参考线的距离，L：垂直偏移）
  // std::pair<std::array<double, 3>, std::array<double, 3>>
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
  // 评估路径并写入 reference_line_info->path_data()
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
  // 创建一个新的 PathBoundary 对象并获取引用
  boundary->emplace_back();
  auto& path_bound = boundary->back();
  
  // 初始化局部变量，用于记录阻塞障碍物等信息
  // 存储障碍物ID
  std::string blocking_obstacle_id = "";
  // 车道类型
  std::string lane_type = "";
  // 路径最窄宽度
  double path_narrowest_width = 0;

  //  先初始化路径边界为一个非常宽的区域，保证后续有空间收窄
  // 1. Initialize the path boundaries to be an indefinitely large area.
  // 步骤1：初始化一个“无限宽”的边界（如 L ∈ [-1e5, 1e5]），作为后续裁剪的基础
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
  // 步骤2：根据自车道（self lane）的左右边界，生成粗略的 L 上下界
  if (!PathBoundsDeciderUtil::GetBoundaryFromSelfLane(
          *reference_line_info_, init_sl_state_, &path_bound)) {
    AERROR << "Failed to decide a rough boundary based on self lane.";
    return false;
  }
 
  // 将车辆占据的空间（含一定缓冲）加入路径边界，避免规划路径侵入车辆自身
  // 若需要，扩展边界确保自车完全包含在内（加 buffer）。
  if (is_include_adc) {
    PathBoundsDeciderUtil::ExtendBoundaryByADC(
        *reference_line_info_, init_sl_state_, config_.extend_buffer(),
        &path_bound);
  }

  // 调试：将所有障碍物在 SL 坐标系下的边界点记录到日志（用于可视化分析）。
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
  // 保存原始边界副本，用于后续补充尾部点
  PathBound temp_path_bound = path_bound;
  // 根据静态障碍物进一步微调路径边界
  // 会选出最远的那个阻塞障碍物id
  // 步骤3：根据静态障碍物收缩边界，避免碰撞
  // blocking_obstacle_id：最近/最窄处的障碍物 ID
  // path_narrowest_width：最窄宽度
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
  // 步骤4：若因障碍物截断导致路径过短，从原始边界中补点（最多 FLAGS_num_extra_tail_bound_point 个）
  int counter = 0;
  while (!blocking_obstacle_id.empty() &&
         path_bound.size() < temp_path_bound.size() &&
         counter < FLAGS_num_extra_tail_bound_point) {  // 20
    path_bound.push_back(temp_path_bound[path_bound.size()]);
    counter++;
  }

  //  更新规划状态中的障碍物阻塞信息
  // lane_follow_status update
  // 获取规划上下文中的 lane_follow 状态对象，用于更新阻塞信息
  auto* lane_follow_status = injector_->planning_context()
                                    ->mutable_planning_status()
                                    ->mutable_lane_follow();
  // 如果有阻塞障碍物
  if (!blocking_obstacle_id.empty()) {
    // 如果有阻塞的障碍物，记录阻塞的障碍物ID，并更新阻塞持续时间
    double current_time = ::apollo::cyber::Clock::NowInSeconds();
    lane_follow_status->set_block_obstacle_id(blocking_obstacle_id);
    // 若之前已阻塞 → 累加阻塞时长
    if (lane_follow_status->lane_follow_block()) {
      lane_follow_status->set_block_duration(
          lane_follow_status->block_duration() + current_time -
          lane_follow_status->last_block_timestamp());
    } else {
    // 否则 → 标记为开始阻塞，重置时长
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
  // 合法性检查：自车初始 L 位置必须在边界内。
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
  // 设置边界标签为 "regular/self"，便于追踪来源。
  path_bound.set_label(absl::StrCat("regular/", "self"));
  // 设置阻塞障碍物 ID
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
  // 硬编码终止状态：(l=0, dl=0, ddl=0) —— 假设终点回到车道中心且平直
  std::array<double, 3> end_state = {0.0, 0.0, 0.0};
  // 遍历所有路径边界: 对每一条候选路径边界执行一次路径优化
  // 遍历所有候选边界（通常只有1个）
  for (const auto& path_boundary : path_boundaries) {
    // 路径边界校验: 确保路径边界的点数量足够（至少两个点），否则报错返回失败
    // 边界点数必须 ≥2，否则无法优化
    size_t path_boundary_size = path_boundary.boundary().size();
    if (path_boundary_size <= 1U) {
      AERROR << "Get invalid path boundary with size: " << path_boundary_size;
      return false;
    }

    // 计算每个 s 点的加速度（ddl）上下界，基于车辆动力学和道路曲率。
    // 定义优化结果容器: 存储优化得到的横向偏移 l、一阶导数 dl（速度）和二阶导数 ddl（曲率变化）
    std::vector<double> opt_l, opt_dl, opt_ddl;
    // 计算曲率二阶导（加速度）的边界：根据路径边界和参考线信息，计算每个点允许的最大曲率变化范围（上下限）
    std::vector<std::pair<double, double>> ddl_bounds;
    PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line,
                                         &ddl_bounds);


    // 调试：记录参考线曲率 κ(s)
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
    
    // 估算 jerk（加加速度）上限，基于初始纵向速度（防止高速时路径抖动）
    // 估计路径平滑性限制（jerk 限制）：通过当前初始状态的横向速度估算路径允许的最大三阶导数（jerk）边界
    const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(
        std::fmax(init_sl_state_.first[1], 1e-12));

    // 构建参考 L 值（通常为0，即车道中心）和对应权重（越靠近中心权重越高）
    // 生成路径参考偏移（ref_l）及其权重：通常以中心线或安全区域为参考，引导优化结果向安全区域居中，同时赋予不同位置不同的优化权重
    std::vector<double> ref_l(path_boundary_size, 0);
    std::vector<double> weight_ref_l(path_boundary_size, 0);
    PathOptimizerUtil::UpdatePathRefWithBound(
        path_boundary, config.path_reference_l_weight(), &ref_l, &weight_ref_l);

    // 核心优化：求解分段 jerk 最小化问题，在边界和动力学约束下生成平滑 L(s)
    // 执行路径优化： 对当前 path_boundary 区域执行最小化 cost 的轨迹规划，结果保存在 opt_l, opt_dl, opt_ddl 中
    bool res_opt = PathOptimizerUtil::OptimizePath(
        init_sl_state_, end_state, ref_l, weight_ref_l, path_boundary,
        ddl_bounds, jerk_bound, config, &opt_l, &opt_dl, &opt_ddl);

    // 将离散优化结果转为 PiecewiseJerkPath（分段三次多项式）
    if (res_opt) {
      // 生成 Frenet 坐标系下的路径结构： 将优化结果转为可用于后续坐标转换与跟踪控制的结构体
      auto frenet_frame_path = PathOptimizerUtil::ToPiecewiseJerkPath(
          opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
          path_boundary.start_s());

      // 构造 PathData 对象
      //  构造路径数据对象 PathData：绑定参考线并设置刚才优化得到的 Frenet 路径
      PathData path_data;
      path_data.SetReferenceLine(&reference_line);
      path_data.SetFrenetPath(std::move(frenet_frame_path));

      // 若配置使用前轴中心，则将路径点转换为后轴中心（车辆控制通常以后轴为参考）
      // 前轴 / 后轴参考点转换：某些规划场景下需要以前轴中心（或后轴中心）为参考点进行路径转换，以匹配控制器模型
      if (FLAGS_use_front_axe_center_in_path_planning) {
        auto discretized_path = DiscretizedPath(
            PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(
                path_data));
        path_data.SetDiscretizedPath(discretized_path);
      }

      // 设置标签和阻塞障碍物，并加入候选列表
      // 设置路径标签和阻塞障碍物 ID
      // path_label：用于标识该路径用途（如主道、换道、临停等）
      // blocking_obstacle_id：路径被谁阻挡了，便于后续决策使用
      path_data.set_path_label(path_boundary.label());
      path_data.set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
      candidate_path_data->push_back(std::move(path_data));
      // 调试：记录生成路径的曲率
      PrintCurves print_path_kappa;
      for (const auto& p : candidate_path_data->back().discretized_path()) {
        print_path_kappa.AddPoint(path_boundary.label() + "_path_kappa",
                                  p.s() + init_sl_state_.first[0], p.kappa());
      }
      print_path_kappa.PrintToLog();
    }
  }
  if (candidate_path_data->empty()) {
    return false;
  }
  return true;
}

bool LaneFollowPath::AssessPath(std::vector<PathData>* candidate_path_data,
                                PathData* final_path) {
  // 取最后一个（当前唯一）路径，记录调试信息。
  PathData& curr_path_data = candidate_path_data->back();
  RecordDebugInfo(curr_path_data, curr_path_data.path_label(),
                  reference_line_info_);
  // 检查路径是否有效（无碰撞、曲率合理、在边界内等）
  if (!PathAssessmentDeciderUtil::IsValidRegularPath(*reference_line_info_,
                                                     curr_path_data)) {
    AINFO << "Lane follow path is invalid";
    return false;
  }
  
  // 为每个路径点打标签（如 IN_LANE），供后续速度规划使用。
  std::vector<PathPointDecision> path_decision;
  PathAssessmentDeciderUtil::InitPathPointDecision(
      curr_path_data, PathData::PathPointType::IN_LANE, &path_decision);
  curr_path_data.SetPathPointDecisionGuide(std::move(path_decision));
  
  // 若裁剪后路径为空，无效。
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
