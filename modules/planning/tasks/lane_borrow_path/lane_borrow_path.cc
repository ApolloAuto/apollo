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

#include "modules/planning/tasks/lane_borrow_path/lane_borrow_path.h"

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/common/obstacle_blocking_analyzer.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

constexpr double kIntersectionClearanceDist = 20.0;
constexpr double kJunctionClearanceDist = 15.0;

bool LaneBorrowPath::Init(const std::string& config_dir,
                          const std::string& name,
                          const std::shared_ptr<DependencyInjector>& injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  return Task::LoadConfig<LaneBorrowPathConfig>(&config_);
}

apollo::common::Status LaneBorrowPath::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  if (!config_.is_allow_lane_borrowing() ||
      reference_line_info->path_reusable()) {
    ADEBUG << "path reusable" << reference_line_info->path_reusable()
           << ",skip";
    return Status::OK();
  }
  if (!IsNecessaryToBorrowLane()) {
    ADEBUG << "No need to borrow lane";
    return Status::OK();
  }
  std::vector<PathBoundary> candidate_path_boundaries;
  std::vector<PathData> candidate_path_data;

  GetStartPointSLState();
  if (!DecidePathBounds(&candidate_path_boundaries)) {
    return Status::OK();
  }
  if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
    return Status::OK();
  }
  if (AssessPath(&candidate_path_data,
                 reference_line_info->mutable_path_data())) {
    ADEBUG << "lane borrow path success";
  }

  return Status::OK();
}

bool LaneBorrowPath::DecidePathBounds(std::vector<PathBoundary>* boundary) {
  for (size_t i = 0; i < decided_side_pass_direction_.size(); i++) {
    boundary->emplace_back();
    auto& path_bound = boundary->back();
    std::string blocking_obstacle_id = "";
    std::string borrow_lane_type = "";
    double path_narrowest_width = 0;
    // 1. Initialize the path boundaries to be an indefinitely large area.
    if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_,
                                                 &path_bound, init_sl_state_)) {
      const std::string msg = "Failed to initialize path boundaries.";
      AERROR << msg;
      boundary->pop_back();
      continue;
    }
    // 2. Decide a rough boundary based on lane info and ADC's position
    if (!GetBoundaryFromNeighborLane(decided_side_pass_direction_[i],
                                     &path_bound, &borrow_lane_type)) {
      AERROR << "Failed to decide a rough boundary based on lane and adc.";
      boundary->pop_back();
      continue;
    }
    // path_bound.DebugString("after_neighbor_lane");
    // 3. Fine-tune the boundary based on static obstacles
    PathBound temp_path_bound = path_bound;
    if (!PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
            *reference_line_info_, init_sl_state_, &path_bound,
            &blocking_obstacle_id, &path_narrowest_width)) {
      const std::string msg =
          "Failed to decide fine tune the boundaries after "
          "taking into consideration all static obstacles.";
      AERROR << msg;
      boundary->pop_back();
      continue;
    }
    // path_bound.DebugString("after_obs");
    // 4. Append some extra path bound points to avoid zero-length path data.
    int counter = 0;
    while (!blocking_obstacle_id.empty() &&
           path_bound.size() < temp_path_bound.size() &&
           counter < FLAGS_num_extra_tail_bound_point) {
      path_bound.push_back(temp_path_bound[path_bound.size()]);
      counter++;
    }
    ADEBUG << "Completed generating path boundaries.";
    std::string label;
    if (decided_side_pass_direction_[i] == SidePassDirection::LEFT_BORROW) {
      label = "regular/left" + borrow_lane_type;
    } else {
      label = "regular/right" + borrow_lane_type;
    }
    path_bound.set_label(label);
    path_bound.set_blocking_obstacle_id(blocking_obstacle_id);
    RecordDebugInfo(path_bound, path_bound.label(), reference_line_info_);
  }
  return !boundary->empty();
}
bool LaneBorrowPath::OptimizePath(
    const std::vector<PathBoundary>& path_boundaries,
    std::vector<PathData>* candidate_path_data) {
  const auto& config = config_.path_optimizer_config();
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  std::array<double, 3> end_state = {0.0, 0.0, 0.0};

  for (const auto& path_boundary : path_boundaries) {
    std::vector<double> opt_l, opt_dl, opt_ddl;
    std::vector<std::pair<double, double>> ddl_bounds;
    PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line,
                                         &ddl_bounds);
    const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(
        std::fmax(init_sl_state_.first[1], 1e-12));
    std::vector<double> ref_l;
    std::vector<double> weight_ref_l;
    PathOptimizerUtil::UpdatePathRefWithBound(
        path_boundary, config.path_reference_l_weight(), &ref_l, &weight_ref_l);

    bool res_opt = PathOptimizerUtil::OptimizePath(
        init_sl_state_, end_state, ref_l, weight_ref_l, path_boundary,
        ddl_bounds, jerk_bound, config, &opt_l, &opt_dl, &opt_ddl);
    if (res_opt) {
      auto frenet_frame_path = PathOptimizerUtil::ToPiecewiseJerkPath(
          opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
          path_boundary.start_s());
      PathData path_data;
      path_data.SetReferenceLine(&reference_line);
      path_data.SetFrenetPath(std::move(frenet_frame_path));
      if (FLAGS_use_front_axe_center_in_path_planning) {
        auto discretized_path = DiscretizedPath(
            PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(
                path_data));
        path_data.SetDiscretizedPath(discretized_path);
      }
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

bool LaneBorrowPath::AssessPath(std::vector<PathData>* candidate_path_data,
                                PathData* final_path) {
  std::vector<PathData> valid_path_data;
  for (auto& curr_path_data : *candidate_path_data) {
    if (PathAssessmentDeciderUtil::IsValidRegularPath(*reference_line_info_,
                                                      curr_path_data)) {
      SetPathInfo(&curr_path_data);
      PathAssessmentDeciderUtil::TrimTailingOutLanePoints(&curr_path_data);
      if (curr_path_data.Empty()) {
        AINFO << "lane borrow path is empty after trimed";
        continue;
      }
      valid_path_data.push_back(curr_path_data);
    }
  }
  if (valid_path_data.empty()) {
    AINFO << "All lane borrow path are not valid";
    return false;
  }
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  const std::string blocking_obstacle_id =
      mutable_path_decider_status->front_static_obstacle_id();
  const Obstacle* blocking_obstacle =
      reference_line_info_->path_decision()->obstacles().Find(
          blocking_obstacle_id);
  if (valid_path_data.size() > 1) {
    if (ComparePathData(valid_path_data[0], valid_path_data[1],
                        blocking_obstacle)) {
      *final_path = valid_path_data[0];
    } else {
      *final_path = valid_path_data[1];
    }
  } else {
    *final_path = valid_path_data[0];
  }
  RecordDebugInfo(*final_path, final_path->path_label(), reference_line_info_);
  return true;
}

bool LaneBorrowPath::GetBoundaryFromNeighborLane(
    const SidePassDirection pass_direction, PathBoundary* const path_bound,
    std::string* borrow_lane_type) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  double adc_lane_width = PathBoundsDeciderUtil::GetADCLaneWidth(
      reference_line, init_sl_state_.first[0]);
  double offset_to_map = 0;
  bool borrowing_reverse_lane = false;
  reference_line.GetOffsetToMap(init_sl_state_.first[0], &offset_to_map);
  // Go through every point, update the boundary based on lane info and
  // ADC's position.
  double past_lane_left_width = adc_lane_width / 2.0;
  double past_lane_right_width = adc_lane_width / 2.0;
  int path_blocked_idx = -1;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = (*path_bound)[i].s;
    // 1. Get the current lane width at current point.
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    double offset_to_lane_center = 0.0;
    if (!reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                     &curr_lane_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_lane_left_width = past_lane_left_width;
      curr_lane_right_width = past_lane_right_width;
    } else {
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;
      past_lane_left_width = curr_lane_left_width;
      past_lane_right_width = curr_lane_right_width;
    }
    // 2. Get the neighbor lane widths at the current point.
    double curr_neighbor_lane_width = 0.0;
    if (CheckLaneBoundaryType(*reference_line_info_, curr_s, pass_direction)) {
      hdmap::Id neighbor_lane_id;
      if (pass_direction == SidePassDirection::LEFT_BORROW) {
        // Borrowing left neighbor lane.
        if (reference_line_info_->GetNeighborLaneInfo(
                ReferenceLineInfo::LaneType::LeftForward, curr_s,
                &neighbor_lane_id, &curr_neighbor_lane_width)) {
          ADEBUG << "Borrow left forward neighbor lane."
                << neighbor_lane_id.id();
        } else if (reference_line_info_->GetNeighborLaneInfo(
                       ReferenceLineInfo::LaneType::LeftReverse, curr_s,
                       &neighbor_lane_id, &curr_neighbor_lane_width)) {
          borrowing_reverse_lane = true;
          ADEBUG << "Borrow left reverse neighbor lane."
                << neighbor_lane_id.id();
        } else {
          ADEBUG << "There is no left neighbor lane.";
        }
      } else if (pass_direction == SidePassDirection::RIGHT_BORROW) {
        // Borrowing right neighbor lane.
        if (reference_line_info_->GetNeighborLaneInfo(
                ReferenceLineInfo::LaneType::RightForward, curr_s,
                &neighbor_lane_id, &curr_neighbor_lane_width)) {
          ADEBUG << "Borrow right forward neighbor lane."
                << neighbor_lane_id.id();
        } else if (reference_line_info_->GetNeighborLaneInfo(
                       ReferenceLineInfo::LaneType::RightReverse, curr_s,
                       &neighbor_lane_id, &curr_neighbor_lane_width)) {
          borrowing_reverse_lane = true;
          ADEBUG << "Borrow right reverse neighbor lane."
                << neighbor_lane_id.id();
        } else {
          ADEBUG << "There is no right neighbor lane.";
        }
      }
    }
    // 3. Calculate the proper boundary based on lane-width, ADC's position,
    //    and ADC's velocity.
    double offset_to_map = 0.0;
    reference_line.GetOffsetToMap(curr_s, &offset_to_map);

    double curr_left_bound_lane =
        curr_lane_left_width + (pass_direction == SidePassDirection::LEFT_BORROW
                                    ? curr_neighbor_lane_width
                                    : 0.0);

    double curr_right_bound_lane =
        -curr_lane_right_width -
        (pass_direction == SidePassDirection::RIGHT_BORROW
             ? curr_neighbor_lane_width
             : 0.0);
    double curr_left_bound = 0.0;
    double curr_right_bound = 0.0;
    curr_left_bound = curr_left_bound_lane - offset_to_map;
    curr_right_bound = curr_right_bound_lane - offset_to_map;

    // 4. Update the boundary.
    if (!PathBoundsDeciderUtil::UpdatePathBoundaryWithBuffer(
            curr_left_bound, curr_right_bound, BoundType::LANE, BoundType::LANE,
            "", "", &path_bound->at(i))) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) {
      break;
    }
  }
  PathBoundsDeciderUtil::TrimPathBounds(path_blocked_idx, path_bound);
  *borrow_lane_type = borrowing_reverse_lane ? "reverse" : "forward";
  return true;
}
void LaneBorrowPath::UpdateSelfPathInfo() {
  auto cur_path = reference_line_info_->path_data();
  if (!cur_path.Empty() &&
      cur_path.path_label().find("self") != std::string::npos &&
      cur_path.blocking_obstacle_id().empty()) {
    use_self_lane_ = std::min(use_self_lane_ + 1, 10);
  } else {
    use_self_lane_ = 0;
  }
  blocking_obstacle_id_ = cur_path.blocking_obstacle_id();
}
/// @brief 判断当前是否有必要借用邻车道（lane borrowing）
/// @return 
bool LaneBorrowPath::IsNecessaryToBorrowLane() {
  // mutable_path_decider_status 是一个指针，指向当前路径决策器的可变状态
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  // 判断当前是否已经处于借用邻车道的场景中。如果是，进入接下来的逻辑                                       
  if (mutable_path_decider_status->is_in_path_lane_borrow_scenario()) {
    // 更新车辆自身的路径信息
    UpdateSelfPathInfo();
    // If originally borrowing neighbor lane:
    if (use_self_lane_ >= 6) {
      // If have been able to use self-lane for some time, then switch to
      // non-lane-borrowing.
      // 这里判断 use_self_lane_ 的值是否大于等于6。如果是，说明车辆已经使用了自车道足够长的时间，可以考虑切换到不借用邻车道的状态
      // 如果条件满足，就更新状态为不借道，并清空已决定的侧向通过方向 (decided_side_pass_direction_)，输出日志
      mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
      decided_side_pass_direction_.clear();
      AINFO << "Switch from LANE-BORROW path to SELF-LANE path.";
    }
  } else {
    // If originally not borrowing neighbor lane:
    // 如果当前不在借道状态中，输出阻塞障碍物的ID
    AINFO << "Blocking obstacle ID["
          << mutable_path_decider_status->front_static_obstacle_id() << "]";
    // ADC requirements check for lane-borrowing:
    // 只有一条参考线，才能借道
    if (!HasSingleReferenceLine(*frame_)) {
      return false;  // 不可以借道
    }
    // 起点速度小于最大借道允许速度
    if (!IsWithinSidePassingSpeedADC(*frame_)) {
      return false;
    }

    // Obstacle condition check for lane-borrowing:
    // 阻塞障碍物是否远离路口
    if (!IsBlockingObstacleFarFromIntersection(*reference_line_info_)) {
      return false;
    }
    // 阻塞障碍物长期存在
    if (!IsLongTermBlockingObstacle()) {
      return false;
    }
    // 阻塞障碍物是否在终点位置与自车间距之内
    if (!IsBlockingObstacleWithinDestination(*reference_line_info_)) {
      return false;
    }
     // 为可侧面通过的障碍物
    if (!IsSidePassableObstacle(*reference_line_info_)) {
      return false;
    }

    // switch to lane-borrowing
    if (decided_side_pass_direction_.empty()) {
      // first time init decided_side_pass_direction
      // 如果 decided_side_pass_direction_ 为空，说明这是第一次判断是否需要借道，需要进行初始化
      bool left_borrowable;
      bool right_borrowable;
      CheckLaneBorrow(*reference_line_info_, &left_borrowable,
                      &right_borrowable);
      if (!left_borrowable && !right_borrowable) {
        mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
        AINFO << "LEFT AND RIGHT LANE CAN NOT BORROW";
        return false;
      } else {
        mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
        if (left_borrowable) {
          decided_side_pass_direction_.push_back(
              SidePassDirection::LEFT_BORROW);
        }
        if (right_borrowable) {
          decided_side_pass_direction_.push_back(
              SidePassDirection::RIGHT_BORROW);
        }
      }
    }
    // 将 use_self_lane_ 重置为0，并输出日志，表示从自车道路径切换到借道路径
    use_self_lane_ = 0;
    AINFO << "Switch from SELF-LANE path to LANE-BORROW path.";
  }
  // 最后返回当前是否处于借道场景的状态
  return mutable_path_decider_status->is_in_path_lane_borrow_scenario();
}

bool LaneBorrowPath::HasSingleReferenceLine(const Frame& frame) {
  return frame.reference_line_info().size() == 1;
}

bool LaneBorrowPath::IsWithinSidePassingSpeedADC(const Frame& frame) {
  return frame.PlanningStartPoint().v() < config_.lane_borrow_max_speed();
}
/// @brief 判断是否存在长期阻塞障碍物
/// @return 
bool LaneBorrowPath::IsLongTermBlockingObstacle() {
  if (injector_->planning_context()
          ->planning_status()
          .path_decider()
          .front_static_obstacle_cycle_counter() >=
      config_.long_term_blocking_obstacle_cycle_threshold()) {   // 3
    ADEBUG << "The blocking obstacle is long-term existing.";
    return true;
  } else {
    ADEBUG << "The blocking obstacle is not long-term existing.";
    return false;
  }
}
/// @brief 判断一个阻塞障碍物是否位于目标区域内
/// @param reference_line_info 
/// @return 
bool LaneBorrowPath::IsBlockingObstacleWithinDestination(
    const ReferenceLineInfo& reference_line_info) {
  // 获取路径决策器（path_decider_status）的状态信息
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  // 如果 blocking_obstacle_id 为空，意味着没有检测到任何阻塞障碍物
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return true;
  }
  // 在路径决策中的障碍物集合中查找指定 ID 的障碍物，结果存储在 blocking_obstacle 指针中
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
 // 如果 blocking_obstacle 为 nullptr，意味着找不到该障碍物，可能它已经被移除或者不再存在
  if (blocking_obstacle == nullptr) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return true;
  }
// 获取该阻塞障碍物的起始位置 s 坐标（通常用于表示障碍物在参考线上的位置）
  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().start_s();
// 获取自动驾驶车辆（ADC）的末端位置 s 坐标
  double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;
  ADEBUG << "ADC is at s = " << adc_end_s;
  ADEBUG << "Destination is at s = "
         << reference_line_info.SDistanceToDestination() + adc_end_s;
  // blocking_obstacle_s - adc_end_s 计算的是障碍物起始位置到 ADC 末端的距离
  // reference_line_info.SDistanceToDestination() 是目标点距离 ADC 末端的距离
  if (blocking_obstacle_s - adc_end_s >
      reference_line_info.SDistanceToDestination()) {
        // 障碍物的起始位置超出了目标区域，则表示该障碍物不在目标区域
    return false;
  }
  return true;
}
/// @brief 判断一个静态障碍物是否阻挡了前方交叉口，如果阻挡物距离交叉口过近，就不允许车辆侧向通过
/// @param reference_line_info 
/// @return 
bool LaneBorrowPath::IsBlockingObstacleFarFromIntersection(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  // 获取前方静态障碍物的ID
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  // 如果 blocking_obstacle_id 为空，说明没有阻挡物
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return true;
  }
  // 查找阻挡物
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  // 如果 blocking_obstacle 为 nullptr，表示阻挡物已经不存在了，输出调试信息并返回 true
  if (blocking_obstacle == nullptr) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return true;
  }

  // Get blocking obstacle's s.
  // 获取阻挡物的感知边界，并获取其结束位置的s坐标（end_s()）
  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().end_s();
  ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;
  // Get intersection's s and compare with threshold.
  // 获取 reference_line_info 中首次遇到的交叉口信息。overlaps 应该包含与路径重叠的其他道路设施（如交通信号、停车标志等）
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  // 遍历 first_encountered_overlaps，对每个重叠物体（交叉口或其他设施）进行处理。overlap.first 是设施类型，overlap.second 是设施的具体信息（可能包含位置、大小等数据）
  for (const auto& overlap : first_encountered_overlaps) {
    ADEBUG << overlap.first << ", " << overlap.second.DebugString();
  // 如果设施不是信号灯或停车标志，跳过当前迭代，继续处理下一个设施
    if (overlap.first != ReferenceLineInfo::SIGNAL &&
        overlap.first != ReferenceLineInfo::STOP_SIGN) {
      continue;
    }
  // 计算从阻挡物的s坐标到设施开始位置的距离 distance
    auto distance = overlap.second.start_s - blocking_obstacle_s;
    if (overlap.first == ReferenceLineInfo::SIGNAL ||
        overlap.first == ReferenceLineInfo::STOP_SIGN) {
          // 根据计算出的距离判断，若距离小于预设的阈值
      if (distance < kIntersectionClearanceDist) {   // 20
        // 表示阻挡物离交叉口太近，无法执行“侧向通过”操作
        ADEBUG << "Too close to signal intersection (" << distance
               << "m); don't SIDE_PASS.";
        return false;
      }
    } else {
      if (distance < kJunctionClearanceDist) {   // 15
        ADEBUG << "Too close to overlap_type[" << overlap.first << "] ("
               << distance << "m); don't SIDE_PASS";
        return false;
      }
    }
  }
 // 如果所有检查通过，说明阻挡物距离交叉口足够远，可以继续侧向通过操作
  return true;
}
/// @brief 
/// @param reference_line_info 
/// @return 
bool LaneBorrowPath::IsSidePassableObstacle(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return false;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (blocking_obstacle == nullptr) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return false;
  }

  return IsNonmovableObstacle(reference_line_info, *blocking_obstacle);
}
/// @brief 
/// @param reference_line_info 
/// @param left_neighbor_lane_borrowable 
/// @param right_neighbor_lane_borrowable 
void LaneBorrowPath::CheckLaneBorrow(
    const ReferenceLineInfo& reference_line_info,
    bool* left_neighbor_lane_borrowable, bool* right_neighbor_lane_borrowable) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();

  *left_neighbor_lane_borrowable = true;
  *right_neighbor_lane_borrowable = true;
  // 定义一个常量kLookforwardDistance，表示检查时需要向前查看的距离
  static constexpr double kLookforwardDistance = 100.0;
  // 返回车辆所在位置的s坐标
  double check_s = reference_line_info.AdcSlBoundary().end_s();
  // 计算实际需要查看的最大前进距离，取当前位置check_s加上kLookforwardDistance和参考线总长度reference_line.Length()中的较小值
  const double lookforward_distance =
      std::min(check_s + kLookforwardDistance, reference_line.Length());
  while (check_s < lookforward_distance) {
    // 获取参考线中与当前s坐标最近的参考点ref_point
    auto ref_point = reference_line.GetNearestReferencePoint(check_s);
    if (ref_point.lane_waypoints().empty()) {
      *left_neighbor_lane_borrowable = false;
      *right_neighbor_lane_borrowable = false;
      return;
    }
    // 通过check_s查找当前参考点对应的车道信息ptr_lane_info
    auto ptr_lane_info = reference_line_info.LocateLaneInfo(check_s);
    if (ptr_lane_info->lane().left_neighbor_forward_lane_id().empty() &&
        ptr_lane_info->lane().left_neighbor_reverse_lane_id().empty()) {
      *left_neighbor_lane_borrowable = false;
    }
    if (ptr_lane_info->lane().right_neighbor_forward_lane_id().empty() &&
        ptr_lane_info->lane().right_neighbor_reverse_lane_id().empty()) {
      *right_neighbor_lane_borrowable = false;
    }
    // 获取当前参考点车道的第一个路标点waypoint
    const auto waypoint = ref_point.lane_waypoints().front();
    // 存储当前路标点的车道边界类型，初始化为UNKNOWN
    hdmap::LaneBoundaryType::Type lane_boundary_type =
        hdmap::LaneBoundaryType::UNKNOWN;

    if (*left_neighbor_lane_borrowable) {
      lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
      if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||  // 实黄线
          lane_boundary_type == hdmap::LaneBoundaryType::DOUBLE_YELLOW ||  // 双黄线
          lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {    // 实白线
        *left_neighbor_lane_borrowable = false;
      }
      ADEBUG << "s[" << check_s << "] left_lane_boundary_type["
             << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
    }
    if (*right_neighbor_lane_borrowable) {
      lane_boundary_type = hdmap::RightBoundaryType(waypoint);
      if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
          lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
        *right_neighbor_lane_borrowable = false;
      }
      ADEBUG << "s[" << check_s << "] right_neighbor_lane_borrowable["
             << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
    }
    // 每次检查完一个位置后，check_s增加2.0米，继续向前检查
    check_s += 2.0;
  }
}

bool LaneBorrowPath::CheckLaneBoundaryType(
    const ReferenceLineInfo& reference_line_info, const double check_s,
    const SidePassDirection& lane_borrow_info) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  auto ref_point = reference_line.GetNearestReferencePoint(check_s);
  if (ref_point.lane_waypoints().empty()) {
    return false;
  }

  const auto waypoint = ref_point.lane_waypoints().front();
  hdmap::LaneBoundaryType::Type lane_boundary_type =
      hdmap::LaneBoundaryType::UNKNOWN;
  if (lane_borrow_info == SidePassDirection::LEFT_BORROW) {
    lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
  } else if (lane_borrow_info == SidePassDirection::RIGHT_BORROW) {
    lane_boundary_type = hdmap::RightBoundaryType(waypoint);
  }
  if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
      lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
    return false;
  }
  return true;
}

void LaneBorrowPath::SetPathInfo(PathData* const path_data) {
  std::vector<PathPointDecision> path_decision;
  PathAssessmentDeciderUtil::InitPathPointDecision(
      *path_data, PathData::PathPointType::IN_LANE, &path_decision);
  // Go through every path_point, and add in-lane/out-of-lane info.
  const auto& discrete_path = path_data->discretized_path();
  bool is_prev_point_out_lane = false;
  SLBoundary ego_sl_boundary;
  for (size_t i = 0; i < discrete_path.size(); ++i) {
    if (!GetSLBoundary(*path_data, i, reference_line_info_, &ego_sl_boundary)) {
      ADEBUG << "Unable to get SL-boundary of ego-vehicle.";
      continue;
    }
    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    double middle_s =
        (ego_sl_boundary.start_s() + ego_sl_boundary.end_s()) / 2.0;
    if (reference_line_info_->reference_line().GetLaneWidth(
            middle_s, &lane_left_width, &lane_right_width)) {
      // Rough sl boundary estimate using single point lane width
      double back_to_inlane_extra_buffer = 0.2;
      double in_and_out_lane_hysteresis_buffer =
          is_prev_point_out_lane ? back_to_inlane_extra_buffer : 0.0;
      // For lane-borrow path, as long as ADC is not on the lane of
      // reference-line, it is out on other lanes. It might even be
      // on reverse lane!
      if (ego_sl_boundary.end_l() >
              lane_left_width + in_and_out_lane_hysteresis_buffer ||
          ego_sl_boundary.start_l() <
              -lane_right_width - in_and_out_lane_hysteresis_buffer) {
        if (path_data->path_label().find("reverse") != std::string::npos) {
          std::get<1>((path_decision)[i]) =
              PathData::PathPointType::OUT_ON_REVERSE_LANE;
        } else if (path_data->path_label().find("forward") !=
                   std::string::npos) {
          std::get<1>((path_decision)[i]) =
              PathData::PathPointType::OUT_ON_FORWARD_LANE;
        } else {
          std::get<1>((path_decision)[i]) = PathData::PathPointType::UNKNOWN;
        }
        if (!is_prev_point_out_lane) {
          if (ego_sl_boundary.end_l() >
                  lane_left_width + back_to_inlane_extra_buffer ||
              ego_sl_boundary.start_l() <
                  -lane_right_width - back_to_inlane_extra_buffer) {
            is_prev_point_out_lane = true;
          }
        }
      } else {
        // The path point is within the reference_line's lane.
        std::get<1>((path_decision)[i]) = PathData::PathPointType::IN_LANE;
        if (is_prev_point_out_lane) {
          is_prev_point_out_lane = false;
        }
      }

    } else {
      AERROR
          << "reference line not ready when setting path point guide, middle_s"
          << middle_s << ",index" << i << "path point"
          << discrete_path[i].DebugString();
      break;
    }
  }
  path_data->SetPathPointDecisionGuide(std::move(path_decision));
}

bool ComparePathData(const PathData& lhs, const PathData& rhs,
                     const Obstacle* blocking_obstacle) {
  ADEBUG << "Comparing " << lhs.path_label() << " and " << rhs.path_label();
  static constexpr double kNeighborPathLengthComparisonTolerance = 25.0;
  double lhs_path_length = lhs.frenet_frame_path().back().s();
  double rhs_path_length = rhs.frenet_frame_path().back().s();
  // Select longer path.
  // If roughly same length, then select self-lane path.
  if (std::fabs(lhs_path_length - rhs_path_length) >
      kNeighborPathLengthComparisonTolerance) {
    return lhs_path_length > rhs_path_length;
  }
  // If roughly same length, and must borrow neighbor lane,
  // then prefer to borrow forward lane rather than reverse lane.
  int lhs_on_reverse =
      ContainsOutOnReverseLane(lhs.path_point_decision_guide());
  int rhs_on_reverse =
      ContainsOutOnReverseLane(rhs.path_point_decision_guide());
  // TODO(jiacheng): make this a flag.
  if (std::abs(lhs_on_reverse - rhs_on_reverse) > 6) {
    return lhs_on_reverse < rhs_on_reverse;
  }
  // For two lane-borrow directions, based on ADC's position,
  // select the more convenient one.
  if (blocking_obstacle) {
    // select left/right path based on blocking_obstacle's position
    const double obstacle_l =
        (blocking_obstacle->PerceptionSLBoundary().start_l() +
         blocking_obstacle->PerceptionSLBoundary().end_l()) /
        2;
    ADEBUG << "obstacle[" << blocking_obstacle->Id() << "] l[" << obstacle_l
           << "]";
    return (obstacle_l > 0.0
                ? (lhs.path_label().find("right") != std::string::npos)
                : (lhs.path_label().find("left") != std::string::npos));
  } else {
    // select left/right path based on ADC's position
    double adc_l = lhs.frenet_frame_path().front().l();
    if (adc_l < -1.0) {
      return lhs.path_label().find("right") != std::string::npos;
    } else if (adc_l > 1.0) {
      return lhs.path_label().find("left") != std::string::npos;
    }
  }
  // If same length, both neighbor lane are forward,
  // then select the one that returns to in-lane earlier.
  static constexpr double kBackToSelfLaneComparisonTolerance = 20.0;
  int lhs_back_idx = GetBackToInLaneIndex(lhs.path_point_decision_guide());
  int rhs_back_idx = GetBackToInLaneIndex(rhs.path_point_decision_guide());
  double lhs_back_s = lhs.frenet_frame_path()[lhs_back_idx].s();
  double rhs_back_s = rhs.frenet_frame_path()[rhs_back_idx].s();
  if (std::fabs(lhs_back_s - rhs_back_s) > kBackToSelfLaneComparisonTolerance) {
    return lhs_back_idx < rhs_back_idx;
  }
  // If same length, both forward, back to inlane at same time,
  // select the left one to side-pass.
  bool lhs_on_leftlane = lhs.path_label().find("left") != std::string::npos;
  return lhs_on_leftlane;
}

int ContainsOutOnReverseLane(
    const std::vector<PathPointDecision>& path_point_decision) {
  int ret = 0;
  for (const auto& curr_decision : path_point_decision) {
    if (std::get<1>(curr_decision) ==
        PathData::PathPointType::OUT_ON_REVERSE_LANE) {
      ++ret;
    }
  }
  return ret;
}

int GetBackToInLaneIndex(
    const std::vector<PathPointDecision>& path_point_decision) {
  // ACHECK(!path_point_decision.empty());
  // ACHECK(std::get<1>(path_point_decision.back()) ==
  //       PathData::PathPointType::IN_LANE);

  for (int i = static_cast<int>(path_point_decision.size()) - 1; i >= 0; --i) {
    if (std::get<1>(path_point_decision[i]) !=
        PathData::PathPointType::IN_LANE) {
      return i;
    }
  }
  return 0;
}

}  // namespace planning
}  // namespace apollo
