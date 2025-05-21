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
 * @brief Implementation of the class ReferenceLineProvider.
 */

#include "modules/planning/planning_base/reference_line/reference_line_provider.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "cyber/common/file.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "cyber/task/task.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

using apollo::common::VehicleConfigHelper;
using apollo::common::VehicleState;
using apollo::common::math::AngleDiff;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::LaneWaypoint;
using apollo::hdmap::MapPathPoint;
using apollo::hdmap::RouteSegments;

ReferenceLineProvider::~ReferenceLineProvider() {}

/// @brief 
/// @param vehicle_state_provider 一个指向 common::VehicleStateProvider 的指针，用于提供车辆状态信息
/// @param reference_line_config 参考线的配置，包含参考线的各种参数
/// @param relative_map 用于导航模式下的相对地图信息
ReferenceLineProvider::ReferenceLineProvider(
    const common::VehicleStateProvider *vehicle_state_provider,
    const ReferenceLineConfig *reference_line_config,
    const std::shared_ptr<relative_map::MapMsg> &relative_map)
    : vehicle_state_provider_(vehicle_state_provider) {
  current_pnc_map_ = nullptr;
  if (!FLAGS_use_navigation_mode) {
    relative_map_ = nullptr;
  } else {
    relative_map_ = relative_map;
  }

  // modules/planning/conf/planning.conf中定义smoother_config_filename
  // 或者使用 modules/planning/common/planning_gflags.cc
  ACHECK(cyber::common::GetProtoFromFile(FLAGS_smoother_config_filename,
                                         &smoother_config_))
      << "Failed to load smoother config file "
      << FLAGS_smoother_config_filename;

  // 根据配置，创建参考线平滑器。 多态: 父类指针指向子类对象   
  if (smoother_config_.has_qp_spline()) {
    smoother_.reset(new QpSplineReferenceLineSmoother(smoother_config_));
  } else if (smoother_config_.has_spiral()) {
    smoother_.reset(new SpiralReferenceLineSmoother(smoother_config_));
  } else if (smoother_config_.has_discrete_points()) {
    smoother_.reset(new DiscretePointsReferenceLineSmoother(smoother_config_));
  } else {
    ACHECK(false) << "unknown smoother config "
                  << smoother_config_.DebugString();
  }
  // Load pnc map plugins.
  pnc_map_list_.clear();
  // Set "apollo::planning::LaneFollowMap" as default if pnc_map_class is empty.
  if (nullptr == reference_line_config ||
      reference_line_config->pnc_map_class().empty()) {
    const auto &pnc_map =
        apollo::cyber::plugin_manager::PluginManager::Instance()
            ->CreateInstance<planning::PncMapBase>(
                "apollo::planning::LaneFollowMap");
    pnc_map_list_.emplace_back(pnc_map);
  } else {
    const auto &pnc_map_names = reference_line_config->pnc_map_class();
    for (const auto &map_name : pnc_map_names) {
      const auto &pnc_map =
          apollo::cyber::plugin_manager::PluginManager::Instance()
              ->CreateInstance<planning::PncMapBase>(map_name);
      pnc_map_list_.emplace_back(pnc_map);
    }
  }

  is_initialized_ = true;
}

/// @brief 更新参考路线的规划命令，并确保当前的规划命令能正确处理并匹配相应的 PNC map（路径导航计算图）
/// @param command 
/// @return 
bool ReferenceLineProvider::UpdatePlanningCommand(
    const planning::PlanningCommand &command) {
 // 使用路由锁保护对 pnc_map_list_ 的访问
  std::lock_guard<std::mutex> routing_lock(routing_mutex_);
  bool find_matched_pnc_map = false;
   // 遍历 pnc_map_list_ 查找能够处理当前命令的 pnc_map
  for (const auto &pnc_map : pnc_map_list_) {
    if (pnc_map->CanProcess(command)) {
      current_pnc_map_ = pnc_map;   // 找到匹配的 pnc_map，更新 current_pnc_map_
      find_matched_pnc_map = true;
      break;
    }
  }
   // 如果没有找到合适的 pnc_map，返回失败
  if (nullptr == current_pnc_map_) {
    AERROR << "Cannot find pnc map to process input command!"
           << command.DebugString();
    return false;
  }
  // 如果没有找到匹配的 pnc_map，警告使用旧的 pnc_map
  if (!find_matched_pnc_map) {
    AWARN << "Find no pnc map for the input command and the old one will be "
             "used!";
  }
  // 锁住 pnc_map_mutex_ 来更新当前 pnc_map 的路由
  // Update routing in pnc_map
  std::lock_guard<std::mutex> lock(pnc_map_mutex_);
  // 如果当前 pnc_map 需要更新规划命令
  if (current_pnc_map_->IsNewPlanningCommand(command)) {
    is_new_command_ = true;

    // 更新 pnc_map 路由信息
    if (!current_pnc_map_->UpdatePlanningCommand(command)) {
      AERROR << "Failed to update routing in pnc map: "
             << command.DebugString();
      return false;
    }
  }
  // 更新当前的规划命令和状态
  planning_command_ = command;
  has_planning_command_ = true;
  return true;
}

std::vector<routing::LaneWaypoint>
ReferenceLineProvider::FutureRouteWaypoints() {
  if (!FLAGS_use_navigation_mode && nullptr != current_pnc_map_) {
    std::lock_guard<std::mutex> lock(pnc_map_mutex_);
    return current_pnc_map_->FutureRouteWaypoints();
  }

  // return an empty routing::LaneWaypoint vector in Navigation mode.
  return std::vector<routing::LaneWaypoint>();
}

/// @brief 获取参考车道的结束航路点，并将其通过引用传递给 end_point
/// @param end_point 
void ReferenceLineProvider::GetEndLaneWayPoint(
    std::shared_ptr<routing::LaneWaypoint> &end_point) const {
  if (nullptr == current_pnc_map_) {
    end_point = nullptr;
    return;
  }
  current_pnc_map_->GetEndLaneWayPoint(end_point);
}

hdmap::LaneInfoConstPtr ReferenceLineProvider::GetLaneById(
    const hdmap::Id &id) const {
  if (nullptr == current_pnc_map_) {
    return nullptr;
  }
  return current_pnc_map_->GetLaneById(id);
}

void ReferenceLineProvider::UpdateVehicleState(
    const VehicleState &vehicle_state) {
  std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
  vehicle_state_ = vehicle_state;
}

/// @brief 
/// @return 
bool ReferenceLineProvider::Start() {
  if (FLAGS_use_navigation_mode) {
    return true;
  }
  if (!is_initialized_) {
    AERROR << "ReferenceLineProvider has NOT been initiated.";
    return false;
  }
  // 异步启动
  // 创建参考线生成线程：FLAGS_enable_reference_line_provider_thread定义在modules/planning/common/pkanning_gflags.cc中
  if (FLAGS_enable_reference_line_provider_thread) {
    task_future_ = cyber::Async(&ReferenceLineProvider::GenerateThread, this);
  }
  return true;
}

void ReferenceLineProvider::Stop() {
  is_stop_ = true;
  if (FLAGS_enable_reference_line_provider_thread) {
    task_future_.get();
  }
}

void ReferenceLineProvider::Reset() {
  std::lock_guard<std::mutex> lock(routing_mutex_);
  has_planning_command_ = false;
  is_new_command_ = false;
  reference_lines_.clear();
  route_segments_.clear();
  is_reference_line_updated_ = false;
  planning_command_.Clear();
  while (!reference_line_history_.empty()) {
    reference_line_history_.pop();
  }
}

void ReferenceLineProvider::UpdateReferenceLine(
    const std::list<ReferenceLine> &reference_lines,
    const std::list<hdmap::RouteSegments> &route_segments) {
  if (reference_lines.size() != route_segments.size()) {
    AERROR << "The calculated reference line size(" << reference_lines.size()
           << ") and route_segments size(" << route_segments.size()
           << ") are different";
    return;
  }
  if (reference_lines.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(reference_lines_mutex_);

  // 将最新计算好的参考线拷贝给成员变量
  if (reference_lines_.size() != reference_lines.size()) {
    reference_lines_ = reference_lines;
    route_segments_ = route_segments;
  } else {
    auto segment_iter = route_segments.begin();
    auto internal_iter = reference_lines_.begin();
    auto internal_segment_iter = route_segments_.begin();
    for (auto iter = reference_lines.begin();
         iter != reference_lines.end() &&
         segment_iter != route_segments.end() &&
         internal_iter != reference_lines_.end() &&
         internal_segment_iter != route_segments_.end();
         ++iter, ++segment_iter, ++internal_iter, ++internal_segment_iter) {
      if (iter->reference_points().empty()) {
        *internal_iter = *iter;  // 将iter所指向的数据复制到internal_iter 所指向的位置
        *internal_segment_iter = *segment_iter;
        continue;
      }
      if (common::util::SamePointXY(
              iter->reference_points().front(),
              internal_iter->reference_points().front()) &&
          common::util::SamePointXY(iter->reference_points().back(),
                                    internal_iter->reference_points().back()) &&
          std::fabs(iter->Length() - internal_iter->Length()) <
              common::math::kMathEpsilon) {
        continue;
      }
      *internal_iter = *iter;
      *internal_segment_iter = *segment_iter;
    }
  }
  // update history
  reference_line_history_.push(reference_lines_);
  route_segments_history_.push(route_segments_);
  static constexpr int kMaxHistoryNum = 3;
  if (reference_line_history_.size() > kMaxHistoryNum) {
    reference_line_history_.pop();
    route_segments_history_.pop();
  }
}

// 多线程的回调函数
void ReferenceLineProvider::GenerateThread() {
  // is_stop_ 是一个 原子变量（std::atomic<bool>），用于标识线程是否应终止
  // 只要 is_stop_ 为 false，线程就会一直运行
  while (!is_stop_) { // 执行的定时任务，每隔50ms提供一次参考线
    static constexpr int32_t kSleepTime = 50;  // milliseconds
    // 线程休眠50ms
    cyber::SleepFor(std::chrono::milliseconds(kSleepTime));
    const double start_time = Clock::NowInSeconds();
    // 如果没有收到新的规划命令，则 跳过本次循环，等待下一次执行
    if (!has_planning_command_) {
      continue;
    }

    // 生成参考线和短期路由
    std::list<ReferenceLine> reference_lines;  // 用于存储生成的参考线
    std::list<hdmap::RouteSegments> segments;  // 用于存储对应的路径段信息
    if (!CreateReferenceLine(&reference_lines, &segments)) {
      is_reference_line_updated_ = false;
      AERROR << "Fail to get reference line";
      continue;
    }


    // 将 reference_lines 和 segments 更新到类成员变量中，以便规划模块使用
    UpdateReferenceLine(reference_lines, segments);
    const double end_time = Clock::NowInSeconds();
    // 使用 互斥锁 reference_lines_mutex_ 保护 last_calculation_time_，防止并发访问导致数据异常
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);
    last_calculation_time_ = end_time - start_time;
    // 新的参考线已更新
    is_reference_line_updated_ = true;
  }
}

double ReferenceLineProvider::LastTimeDelay() {
  if (FLAGS_enable_reference_line_provider_thread &&
      !FLAGS_use_navigation_mode) {
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);
    return last_calculation_time_;
  } else {
    return last_calculation_time_;
  }
}
/// @brief 是否成功获取了参考线（reference_lines）和路段段（segments）
/// @param reference_lines 
/// @param segments 
/// @return 
bool ReferenceLineProvider::GetReferenceLines(
    std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {
// 确保 reference_lines 和 segments 这两个指针不为空
  CHECK_NOTNULL(reference_lines);
  CHECK_NOTNULL(segments);
  if (!has_planning_command_) {
    // 没有必要更新参考线
    return true;
  }
  if (FLAGS_use_navigation_mode) {
    double start_time = Clock::NowInSeconds();
    // 尝试从“相对地图”中获取参考线和路段
    bool result = GetReferenceLinesFromRelativeMap(reference_lines, segments);
    if (!result) {
      AERROR << "Failed to get reference line from relative map";
    }
    double end_time = Clock::NowInSeconds();
    last_calculation_time_ = end_time - start_time;
    return result;
  }

  if (FLAGS_enable_reference_line_provider_thread) {
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);
    if (!reference_lines_.empty()) {
      reference_lines->assign(reference_lines_.begin(), reference_lines_.end());
      segments->assign(route_segments_.begin(), route_segments_.end());
      return true;
    }
  } else {
    double start_time = Clock::NowInSeconds();
    if (CreateReferenceLine(reference_lines, segments)) {
      UpdateReferenceLine(*reference_lines, *segments);
      double end_time = Clock::NowInSeconds();
      last_calculation_time_ = end_time - start_time;
      return true;
    }
  }

  AINFO << "Reference line is NOT ready.";
  if (reference_line_history_.empty()) {
    AINFO << "Failed to use reference line latest history";
    return false;
  }

  reference_lines->assign(reference_line_history_.back().begin(),
                          reference_line_history_.back().end());
  segments->assign(route_segments_history_.back().begin(),
                   route_segments_history_.back().end());
  AWARN << "Use reference line from history!";
  return true;
}

void ReferenceLineProvider::PrioritizeChangeLane(
    std::list<hdmap::RouteSegments> *route_segments) {
  CHECK_NOTNULL(route_segments);
  auto iter = route_segments->begin();
  while (iter != route_segments->end()) {
    if (!iter->IsOnSegment()) {
      route_segments->splice(route_segments->begin(), *route_segments, iter);
      break;
    }
    ++iter;
  }
}
/// @brief 从relative_map（一个描述相对位置的地图）中获取参考路线，并填充到reference_lines和segments列表中
/// @param reference_lines 存储返回的参考线
/// @param segments 存储路径段信息
/// @return 
bool ReferenceLineProvider::GetReferenceLinesFromRelativeMap(
    std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {
  CHECK_GE(relative_map_->navigation_path_size(), 0);
  CHECK_NOTNULL(reference_lines);
  CHECK_NOTNULL(segments);

  if (relative_map_->navigation_path().empty()) {
    AERROR << "There isn't any navigation path in current relative map.";
    return false;
  }

  auto *hdmap = HDMapUtil::BaseMapPtr(*relative_map_);
  if (!hdmap) {
    AERROR << "hdmap is null";
    return false;
  }

  // 1.get adc current lane info ,such as lane_id,lane_priority,neighbor lanes
  //获取当前adc（自动驾驶车辆）的车道信息，首先通过遍历relative_map_中的导航路径，将所有车道ID存入navigation_lane_ids集合
  std::unordered_set<std::string> navigation_lane_ids;
  for (const auto &path_pair : relative_map_->navigation_path()) {
    const auto lane_id = path_pair.first;
    navigation_lane_ids.insert(lane_id);
  }
  // 如果导航路径中的车道ID集合为空，则输出错误日志
  if (navigation_lane_ids.empty()) {
    AERROR << "navigation path ids is empty";
    return false;
  }
  // get current adc lane info by vehicle state
  common::VehicleState vehicle_state = vehicle_state_provider_->vehicle_state();
  hdmap::LaneWaypoint adc_lane_way_point;
  // 根据车辆状态和导航路径获取与当前车道最近的路点（adc_lane_way_point
  if (!GetNearestWayPointFromNavigationPath(vehicle_state, navigation_lane_ids,
                                            &adc_lane_way_point)) {
    return false;
  }
  const std::string adc_lane_id = adc_lane_way_point.lane->id().id();
  auto *adc_navigation_path = apollo::common::util::FindOrNull(
      relative_map_->navigation_path(), adc_lane_id);
  if (adc_navigation_path == nullptr) {
    AERROR << "adc lane cannot be found in relative_map_->navigation_path";
    return false;
  }
  const uint32_t adc_lane_priority = adc_navigation_path->path_priority();
  // get adc left neighbor lanes
  std::vector<std::string> left_neighbor_lane_ids;
  auto left_lane_ptr = adc_lane_way_point.lane;
  while (left_lane_ptr != nullptr &&
         left_lane_ptr->lane().left_neighbor_forward_lane_id_size() > 0) {
    auto neighbor_lane_id =
        left_lane_ptr->lane().left_neighbor_forward_lane_id(0);
    left_neighbor_lane_ids.emplace_back(neighbor_lane_id.id());
    left_lane_ptr = hdmap->GetLaneById(neighbor_lane_id);
  }
  ADEBUG << adc_lane_id
         << " left neighbor size : " << left_neighbor_lane_ids.size();
  for (const auto &neighbor : left_neighbor_lane_ids) {
    ADEBUG << adc_lane_id << " left neighbor : " << neighbor;
  }
  // get adc right neighbor lanes
  std::vector<std::string> right_neighbor_lane_ids;
  auto right_lane_ptr = adc_lane_way_point.lane;
  while (right_lane_ptr != nullptr &&
         right_lane_ptr->lane().right_neighbor_forward_lane_id_size() > 0) {
    auto neighbor_lane_id =
        right_lane_ptr->lane().right_neighbor_forward_lane_id(0);
    right_neighbor_lane_ids.emplace_back(neighbor_lane_id.id());
    right_lane_ptr = hdmap->GetLaneById(neighbor_lane_id);
  }
  ADEBUG << adc_lane_id
         << " right neighbor size : " << right_neighbor_lane_ids.size();
  for (const auto &neighbor : right_neighbor_lane_ids) {
    ADEBUG << adc_lane_id << " right neighbor : " << neighbor;
  }
  // 2.get the higher priority lane info list which priority higher
  // than current lane and get the highest one as the target lane
  using LaneIdPair = std::pair<std::string, uint32_t>;
  std::vector<LaneIdPair> high_priority_lane_pairs;
  ADEBUG << "relative_map_->navigation_path_size = "
         << relative_map_->navigation_path_size();
  for (const auto &path_pair : relative_map_->navigation_path()) {
    const auto lane_id = path_pair.first;
    const uint32_t priority = path_pair.second.path_priority();
    ADEBUG << "lane_id = " << lane_id << " priority = " << priority
           << " adc_lane_id = " << adc_lane_id
           << " adc_lane_priority = " << adc_lane_priority;
    // the smaller the number, the higher the priority
    if (adc_lane_id != lane_id && priority < adc_lane_priority) {
      high_priority_lane_pairs.emplace_back(lane_id, priority);
    }
  }
  // get the target lane
  bool is_lane_change_needed = false;
  LaneIdPair target_lane_pair;
  if (!high_priority_lane_pairs.empty()) {
    std::sort(high_priority_lane_pairs.begin(), high_priority_lane_pairs.end(),
              [](const LaneIdPair &left, const LaneIdPair &right) {
                return left.second < right.second;
              });
    ADEBUG << "need to change lane";
    // the highest priority lane as the target navigation lane
    target_lane_pair = high_priority_lane_pairs.front();
    is_lane_change_needed = true;
  }
  // 3.get current lane's the nearest neighbor lane to the target lane
  // and make sure it position is left or right on the current lane
  routing::ChangeLaneType lane_change_type = routing::FORWARD;
  std::string nearest_neighbor_lane_id;
  if (is_lane_change_needed) {
    // target on the left of adc
    if (left_neighbor_lane_ids.end() !=
        std::find(left_neighbor_lane_ids.begin(), left_neighbor_lane_ids.end(),
                  target_lane_pair.first)) {
      // take the id of the first adjacent lane on the left of adc as
      // the nearest_neighbor_lane_id
      lane_change_type = routing::LEFT;
      nearest_neighbor_lane_id =
          adc_lane_way_point.lane->lane().left_neighbor_forward_lane_id(0).id();
    } else if (right_neighbor_lane_ids.end() !=
               std::find(right_neighbor_lane_ids.begin(),
                         right_neighbor_lane_ids.end(),
                         target_lane_pair.first)) {
      // target lane on the right of adc
      // take the id  of the first adjacent lane on the right of adc as
      // the nearest_neighbor_lane_id
      lane_change_type = routing::RIGHT;
      nearest_neighbor_lane_id = adc_lane_way_point.lane->lane()
                                     .right_neighbor_forward_lane_id(0)
                                     .id();
    }
  }

  for (const auto &path_pair : relative_map_->navigation_path()) {
    const auto &lane_id = path_pair.first;
    const auto &path_points = path_pair.second.path().path_point();
    auto lane_ptr = hdmap->GetLaneById(hdmap::MakeMapId(lane_id));
    RouteSegments segment;
    segment.emplace_back(lane_ptr, 0.0, lane_ptr->total_length());
    segment.SetCanExit(true);
    segment.SetId(lane_id);
    segment.SetNextAction(routing::FORWARD);
    segment.SetStopForDestination(false);
    segment.SetPreviousAction(routing::FORWARD);

    if (is_lane_change_needed) {
      if (lane_id == nearest_neighbor_lane_id) {
        ADEBUG << "adc lane_id = " << adc_lane_id
               << " nearest_neighbor_lane_id = " << lane_id;
        segment.SetIsNeighborSegment(true);
        segment.SetPreviousAction(lane_change_type);
      } else if (lane_id == adc_lane_id) {
        segment.SetIsOnSegment(true);
        segment.SetNextAction(lane_change_type);
      }
    }

    segments->emplace_back(segment);
    std::vector<ReferencePoint> ref_points;
    for (const auto &path_point : path_points) {
      ref_points.emplace_back(
          MapPathPoint{Vec2d{path_point.x(), path_point.y()},
                       path_point.theta(),
                       LaneWaypoint(lane_ptr, path_point.s())},
          path_point.kappa(), path_point.dkappa());
    }
    reference_lines->emplace_back(ref_points.begin(), ref_points.end());
    reference_lines->back().SetPriority(path_pair.second.path_priority());
  }
  return !segments->empty();
}

bool ReferenceLineProvider::GetNearestWayPointFromNavigationPath(
    const common::VehicleState &state,
    const std::unordered_set<std::string> &navigation_lane_ids,
    hdmap::LaneWaypoint *waypoint) {
  const double kMaxDistance = 10.0;
  waypoint->lane = nullptr;
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  auto point = common::util::PointFactory::ToPointENU(state);
  if (std::isnan(point.x()) || std::isnan(point.y())) {
    AERROR << "vehicle state is invalid";
    return false;
  }
  auto *hdmap = HDMapUtil::BaseMapPtr();
  if (!hdmap) {
    AERROR << "hdmap is null";
    return false;
  }

  // get all adc direction lanes from map in kMaxDistance range
  // by vehicle point in map
  const int status = hdmap->GetLanesWithHeading(
      point, kMaxDistance, state.heading(), M_PI / 2.0, &lanes);
  if (status < 0) {
    AERROR << "failed to get lane from point " << point.ShortDebugString();
    return false;
  }

  // get lanes that exist in both map and navigation paths as valid lanes
  std::vector<hdmap::LaneInfoConstPtr> valid_lanes;
  std::copy_if(lanes.begin(), lanes.end(), std::back_inserter(valid_lanes),
               [&](hdmap::LaneInfoConstPtr ptr) {
                 return navigation_lane_ids.count(ptr->lane().id().id()) > 0;
               });
  if (valid_lanes.empty()) {
    AERROR << "no valid lane found within " << kMaxDistance
           << " meters with heading " << state.heading();
    return false;
  }

  // get nearest lane waypoints for current adc position
  double min_distance = std::numeric_limits<double>::infinity();
  for (const auto &lane : valid_lanes) {
    // project adc point to lane to check if it is out of lane range
    double s = 0.0;
    double l = 0.0;
    if (!lane->GetProjection({point.x(), point.y()}, state.heading(), &s, &l)) {
      continue;
    }
    static constexpr double kEpsilon = 1e-6;
    if (s > (lane->total_length() + kEpsilon) || (s + kEpsilon) < 0.0) {
      continue;
    }

    // get the nearest distance between adc point and lane
    double distance = 0.0;
    common::PointENU map_point =
        lane->GetNearestPoint({point.x(), point.y()}, &distance);
    // record the near distance lane
    if (distance < min_distance) {
      double s = 0.0;
      double l = 0.0;
      if (!lane->GetProjection({map_point.x(), map_point.y()}, &s, &l)) {
        AERROR << "failed to get projection for map_point "
               << map_point.DebugString();
        continue;
      }
      min_distance = distance;
      waypoint->lane = lane;
      waypoint->s = s;
    }
  }

  if (waypoint->lane == nullptr) {
    AERROR << "failed to find nearest point " << point.ShortDebugString();
  }
  return waypoint->lane != nullptr;
}

/// @brief 是否成功创建了路线段（Route Segments）
/// @param vehicle_state common::VehicleState 类型，表示车辆的当前状态（如位置、速度、朝向等）
/// @param segments 指向 std::list<hdmap::RouteSegments> 的指针，用于存储生成的路线段
/// @return 
bool ReferenceLineProvider::CreateRouteSegments(
    const common::VehicleState &vehicle_state,
    std::list<hdmap::RouteSegments> *segments) {
  {
    // 使用 std::lock_guard 加锁 pnc_map_mutex_ 互斥量，防止多线程并发访问 pnc_map_ 对象导致数据竞争
    std::lock_guard<std::mutex> lock(pnc_map_mutex_);
    // 根据 vehicle_state 获取当前车辆对应的路线段，并存入 segments
    if (!current_pnc_map_->GetRouteSegments(vehicle_state, segments)) {
      AERROR << "Failed to extract segments from routing";
      return false;
    }
  }
  // 遍历 segments 中的每个 RouteSegment，并调用其 DebugString() 方法，打印调试信息
  for (auto &seg : *segments) {
    ADEBUG << seg.DebugString();
  }
  if (FLAGS_prioritize_change_lane) {
    PrioritizeChangeLane(segments);
  }
  return !segments->empty();
}

/// @brief 根据车辆状态和路由信息生成参考线，并对参考线进行平滑处理或拼接
/// @param reference_lines 用于存储生成的参考线
/// @param segments 用于存储 路由段信息（路径片段）
/// @return 
bool ReferenceLineProvider::CreateReferenceLine(
    std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {
  CHECK_NOTNULL(reference_lines);
  CHECK_NOTNULL(segments);

  // vehicle_state_ 是 ReferenceLineProvider 类的成员变量，存储车辆的 当前位置、速度、航向角等信息
  common::VehicleState vehicle_state;  // .pb.h
  {
    std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
    vehicle_state = vehicle_state_;
  }

  // planning_command_ 存储了规划模块的指令，可能包含 变道、停止、加速等决策信息
  planning::PlanningCommand command;
  {
    std::lock_guard<std::mutex> lock(routing_mutex_);
    command = planning_command_;
  }
  if (nullptr == current_pnc_map_) {
    AERROR << "Current pnc map is null! " << command.DebugString();
    return false;
  }
  
  // 根据 当前车辆状态 生成 路径段信息（Route Segments）
  if (!CreateRouteSegments(vehicle_state, segments)) {
    AERROR << "Failed to create reference line from routing";
    return false;
  }

  // 判断是否需要进行参考线拼接
  // 如果 收到新的规划命令 或 禁用拼接模式，则执行 重新生成参考线 逻辑；否则执行 参考线拼接 逻辑
  if (is_new_command_ || !FLAGS_enable_reference_line_stitching) {  // 首次运行或者新路由，不拼接参考线
    // 重新生成参考线
    // 遍历路径段 segments，为每个路径段创建一个参考线
    for (auto iter = segments->begin(); iter != segments->end();) {
      reference_lines->emplace_back();

  // 参考线平滑
  // 平滑各路由片段列表segments，并将平滑后的路由片段存储到参考线reference_lines中，同时将不能平滑的路由片段从segments中删除
      if (!SmoothRouteSegment(*iter, &reference_lines->back())) {
        AERROR << "Failed to create reference line from route segments";
        // 失败，删除该路径段
        reference_lines->pop_back();
        iter = segments->erase(iter);
      } else {
        common::SLPoint sl;
        // 车辆坐标投影到参考线的 Frenet 坐标系
        if (!reference_lines->back().XYToSL(
                vehicle_state.heading(),
                common::math::Vec2d(vehicle_state.x(), vehicle_state.y()),
                &sl)) {
          AWARN << "Failed to project point: {" << vehicle_state.x() << ","
                << vehicle_state.y() << "} to stitched reference line";
        }
        // 成功，保留该参考线
        // 用于裁剪参考线，确保它符合当前规划需求
        Shrink(sl, &reference_lines->back(), &(*iter));  // 收缩参考线
        ++iter;
      }
    }
    is_new_command_ = false;
    return true;
  } else {  // stitching reference line
  // 启用了参考线拼接（stitching）
  // 先遍历 segments，为每个路径段创建新的参考线
    for (auto iter = segments->begin(); iter != segments->end();) {
      reference_lines->emplace_back();

// 合并不同参考片段的重合部分，并将拼接后的路由片段保存到参考线列表reference_lines中，同时将不能拼接的路由片段从segments中删除
      // 负责将新路径段拼接到现有参考线上
      if (!ExtendReferenceLine(vehicle_state, &(*iter),
                               &reference_lines->back())) {
        AERROR << "Failed to extend reference line";
        reference_lines->pop_back();
        iter = segments->erase(iter);
      } else {
        ++iter;
      }
    }
  }
  return true;
}

bool ReferenceLineProvider::ExtendReferenceLine(const VehicleState &state,
                                                RouteSegments *segments,
                                                ReferenceLine *reference_line) {
  RouteSegments segment_properties;
  segment_properties.SetProperties(*segments);
  auto prev_segment = route_segments_.begin();
  auto prev_ref = reference_lines_.begin();
  while (prev_segment != route_segments_.end()) {
    if (prev_segment->IsConnectedSegment(*segments)) {
      break;
    }
    ++prev_segment;
    ++prev_ref;
  }
  if (prev_segment == route_segments_.end()) {
    if (!route_segments_.empty() && segments->IsOnSegment()) {
      AWARN << "Current route segment is not connected with previous route "
               "segment";
    }
    return SmoothRouteSegment(*segments, reference_line);
  }
  common::SLPoint sl_point;
  Vec2d vec2d(state.x(), state.y());
  LaneWaypoint waypoint;
  if (!prev_segment->GetProjection(vec2d, state.heading(), &sl_point,
                                   &waypoint)) {
    AWARN << "Vehicle current point: " << vec2d.DebugString()
          << " not on previous reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }
  const double prev_segment_length = RouteSegments::Length(*prev_segment);
  const double remain_s = prev_segment_length - sl_point.s();
  const double look_forward_required_distance =
      planning::PncMapBase::LookForwardDistance(state.linear_velocity());
  if (remain_s > look_forward_required_distance) {
    *segments = *prev_segment;
    segments->SetProperties(segment_properties);
    *reference_line = *prev_ref;
    ADEBUG << "Reference line remain " << remain_s
           << ", which is more than required " << look_forward_required_distance
           << " and no need to extend";
    return true;
  }
  double future_start_s =
      std::max(sl_point.s(), prev_segment_length -
                                 FLAGS_reference_line_stitch_overlap_distance);
  double future_end_s =
      prev_segment_length + FLAGS_look_forward_extend_distance;
  RouteSegments shifted_segments;
  std::unique_lock<std::mutex> lock(pnc_map_mutex_);
  if (!current_pnc_map_->ExtendSegments(*prev_segment, future_start_s,
                                        future_end_s, &shifted_segments)) {
    lock.unlock();
    AERROR << "Failed to shift route segments forward";
    return SmoothRouteSegment(*segments, reference_line);
  }
  lock.unlock();
  if (prev_segment->IsWaypointOnSegment(shifted_segments.LastWaypoint())) {
    *segments = *prev_segment;
    segments->SetProperties(segment_properties);
    *reference_line = *prev_ref;
    ADEBUG << "Could not further extend reference line";
    return true;
  }
  hdmap::Path path(shifted_segments);
  ReferenceLine new_ref(path);
  if (!SmoothPrefixedReferenceLine(*prev_ref, new_ref, reference_line)) {
    AWARN << "Failed to smooth forward shifted reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }
  if (!reference_line->Stitch(*prev_ref)) {
    AWARN << "Failed to stitch reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }
  if (!shifted_segments.Stitch(*prev_segment)) {
    AWARN << "Failed to stitch route segments";
    return SmoothRouteSegment(*segments, reference_line);
  }
  *segments = shifted_segments;
  segments->SetProperties(segment_properties);
  common::SLPoint sl;
  if (!reference_line->XYToSL(state.heading(), vec2d, &sl)) {
    AWARN << "Failed to project point: " << vec2d.DebugString()
          << " to stitched reference line";
  }
  return Shrink(sl, reference_line, segments);
}

bool ReferenceLineProvider::Shrink(const common::SLPoint &sl,
                                   ReferenceLine *reference_line,
                                   RouteSegments *segments) {
  // shrink reference line
  double new_backward_distance = sl.s();
  double new_forward_distance = reference_line->Length() - sl.s();
  bool need_shrink = false;
  if (sl.s() > planning::FLAGS_look_backward_distance * 1.5) {
    ADEBUG << "reference line back side is " << sl.s()
           << ", shrink reference line: origin length: "
           << reference_line->Length();
    new_backward_distance = planning::FLAGS_look_backward_distance;
    need_shrink = true;
  }
  // check heading
  const auto index = reference_line->GetNearestReferenceIndex(sl.s());
  const auto &ref_points = reference_line->reference_points();
  const double cur_heading = ref_points[index].heading();
  auto last_index = index;
  while (last_index < ref_points.size() &&
         std::fabs(AngleDiff(cur_heading, ref_points[last_index].heading())) <
             FLAGS_referfece_line_max_forward_heading_diff) {
    ++last_index;
  }
  --last_index;
  if (last_index != ref_points.size() - 1) {
    need_shrink = true;
    common::SLPoint forward_sl;
    reference_line->XYToSL(ref_points[last_index], &forward_sl);
    new_forward_distance = forward_sl.s() - sl.s();
  }

  // check backward heading
  last_index = index;
  while (last_index > 0 &&
         abs(AngleDiff(cur_heading, ref_points[last_index].heading())) <
             FLAGS_referfece_line_max_backward_heading_diff) {
    --last_index;
  }
  if (last_index != 0) {
    need_shrink = true;
    common::SLPoint backward_sl;
    reference_line->XYToSL(ref_points[last_index], &backward_sl);
    new_backward_distance = sl.s() - backward_sl.s();
  }

  if (need_shrink) {
    if (!reference_line->Segment(sl.s(), new_backward_distance,
                                 new_forward_distance)) {
      AWARN << "Failed to shrink reference line";
    }
    if (!segments->Shrink(sl.s(), new_backward_distance,
                          new_forward_distance)) {
      AWARN << "Failed to shrink route segment";
    }
  }
  return true;
}

bool ReferenceLineProvider::IsReferenceLineSmoothValid(
    const ReferenceLine &raw, const ReferenceLine &smoothed) const {
  static constexpr double kReferenceLineDiffCheckStep = 10.0;
  for (double s = 0.0; s < smoothed.Length();
       s += kReferenceLineDiffCheckStep) {
    auto xy_new = smoothed.GetReferencePoint(s);

    common::SLPoint sl_new;
    if (!raw.XYToSL(xy_new, &sl_new)) {
      AERROR << "Fail to change xy point on smoothed reference line to sl "
                "point respect to raw reference line.";
      return false;
    }

    const double diff = std::fabs(sl_new.l());
    if (diff > FLAGS_smoothed_reference_line_max_diff) {
      AERROR << "Fail to provide reference line because too large diff "
                "between smoothed and raw reference lines. diff: "
             << diff;
      return false;
    }
  }
  return true;
}

AnchorPoint ReferenceLineProvider::GetAnchorPoint(
    const ReferenceLine &reference_line, double s) const {
  // 1.设置纵向边界限制
  AnchorPoint anchor;
  anchor.longitudinal_bound = smoother_config_.longitudinal_boundary_bound();

  // 2. 获取参考点
  auto ref_point = reference_line.GetReferencePoint(s);
  if (ref_point.lane_waypoints().empty()) {
    anchor.path_point = ref_point.ToPathPoint(s);
    anchor.lateral_bound = smoother_config_.max_lateral_boundary_bound();
    return anchor;
  }

  const double adc_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width();
  const Vec2d left_vec =
      Vec2d::CreateUnitVec2d(ref_point.heading() + M_PI / 2.0);
  auto waypoint = ref_point.lane_waypoints().front();
  double left_width = 0.0;
  double right_width = 0.0;
  waypoint.lane->GetWidth(waypoint.s, &left_width, &right_width);
  const double kEpislon = 1e-8;
  double effective_width = 0.0;

  // shrink width by vehicle width, curb
  double safe_lane_width = left_width + right_width;
  safe_lane_width -= adc_width;
  bool is_lane_width_safe = true;

  if (safe_lane_width < kEpislon) {
    ADEBUG << "lane width [" << left_width + right_width << "] "
           << "is smaller than adc width [" << adc_width << "]";
    effective_width = kEpislon;
    is_lane_width_safe = false;
  }

  double center_shift = 0.0;
  if (hdmap::RightBoundaryType(waypoint) == hdmap::LaneBoundaryType::CURB) {
    safe_lane_width -= smoother_config_.curb_shift();
    if (safe_lane_width < kEpislon) {
      ADEBUG << "lane width smaller than adc width and right curb shift";
      effective_width = kEpislon;
      is_lane_width_safe = false;
    } else {
      center_shift += 0.5 * smoother_config_.curb_shift();
    }
  }
  if (hdmap::LeftBoundaryType(waypoint) == hdmap::LaneBoundaryType::CURB) {
    safe_lane_width -= smoother_config_.curb_shift();
    if (safe_lane_width < kEpislon) {
      ADEBUG << "lane width smaller than adc width and left curb shift";
      effective_width = kEpislon;
      is_lane_width_safe = false;
    } else {
      center_shift -= 0.5 * smoother_config_.curb_shift();
    }
  }

  //  apply buffer if possible
  const double buffered_width =
      safe_lane_width - 2.0 * smoother_config_.lateral_buffer();
  safe_lane_width =
      buffered_width < kEpislon ? safe_lane_width : buffered_width;

  // shift center depending on the road width
  if (is_lane_width_safe) {
    effective_width = 0.5 * safe_lane_width;
  }

  ref_point += left_vec * center_shift;
  anchor.path_point = ref_point.ToPathPoint(s);
  anchor.lateral_bound = common::math::Clamp(
      effective_width, smoother_config_.min_lateral_boundary_bound(),
      smoother_config_.max_lateral_boundary_bound());
  return anchor;
}

void ReferenceLineProvider::GetAnchorPoints(
    const ReferenceLine &reference_line,
    std::vector<AnchorPoint> *anchor_points) const {
  CHECK_NOTNULL(anchor_points);
// interval为采样间隔，默认max_constraint_interval=0.25，即路径累积距离0.25m采样一个点
  const double interval = smoother_config_.max_constraint_interval();

// 路径采样点数量计算
  int num_of_anchors =
      std::max(2, static_cast<int>(reference_line.Length() / interval + 0.5));

// uniform_slice函数就是对[0, len]区间等间隔采样，每两个点之间距离为(length_ - 0.0)/(num_of_anchors - 1)
  std::vector<double> anchor_s;
  common::util::uniform_slice(0.0, reference_line.Length(), num_of_anchors - 1,
                              &anchor_s);

// 计算采样点的坐标(x,y),并进行矫正
  for (const double s : anchor_s) {
    AnchorPoint anchor = GetAnchorPoint(reference_line, s);
    anchor_points->emplace_back(anchor);
  }

  // 参考线首位两点设置强约束
  anchor_points->front().longitudinal_bound = 1e-6;
  anchor_points->front().lateral_bound = 1e-6;
  anchor_points->front().enforced = true;
  anchor_points->back().longitudinal_bound = 1e-6;
  anchor_points->back().lateral_bound = 1e-6;
  anchor_points->back().enforced = true;
}

bool ReferenceLineProvider::SmoothRouteSegment(const RouteSegments &segments,
                                               ReferenceLine *reference_line) {
  hdmap::Path path(segments);
  return SmoothReferenceLine(ReferenceLine(path), reference_line);
}

bool ReferenceLineProvider::SmoothPrefixedReferenceLine(
    const ReferenceLine &prefix_ref, const ReferenceLine &raw_ref,
    ReferenceLine *reference_line) {
  if (!FLAGS_enable_smooth_reference_line) {
    *reference_line = raw_ref;
    return true;
  }
  // generate anchor points:
  std::vector<AnchorPoint> anchor_points;
  GetAnchorPoints(raw_ref, &anchor_points);
  // modify anchor points based on prefix_ref
  for (auto &point : anchor_points) {
    common::SLPoint sl_point;
    if (!prefix_ref.XYToSL(point.path_point, &sl_point)) {
      continue;
    }
    if (sl_point.s() < 0 || sl_point.s() > prefix_ref.Length()) {
      continue;
    }
    auto prefix_ref_point = prefix_ref.GetNearestReferencePoint(sl_point.s());
    point.path_point.set_x(prefix_ref_point.x());
    point.path_point.set_y(prefix_ref_point.y());
    point.path_point.set_z(0.0);
    point.path_point.set_theta(prefix_ref_point.heading());
    point.longitudinal_bound = 1e-6;
    point.lateral_bound = 1e-6;
    point.enforced = true;
    break;
  }

  smoother_->SetAnchorPoints(anchor_points);
  if (!smoother_->Smooth(raw_ref, reference_line)) {
    AERROR << "Failed to smooth prefixed reference line with anchor points";
    return false;
  }
  if (!IsReferenceLineSmoothValid(raw_ref, *reference_line)) {
    AERROR << "The smoothed reference line error is too large";
    return false;
  }
  return true;
}

bool ReferenceLineProvider::SmoothReferenceLine(
    const ReferenceLine &raw_reference_line, ReferenceLine *reference_line) {
  if (!FLAGS_enable_smooth_reference_line) {
    *reference_line = raw_reference_line;
    return true;
  }
  // generate anchor points:
  std::vector<AnchorPoint> anchor_points;
  GetAnchorPoints(raw_reference_line, &anchor_points);
  smoother_->SetAnchorPoints(anchor_points);
  // 调用具体的平滑算法进行参考线平滑
  if (!smoother_->Smooth(raw_reference_line, reference_line)) {
    AERROR << "Failed to smooth reference line with anchor points";
    return false;
  }
  if (!IsReferenceLineSmoothValid(raw_reference_line, *reference_line)) {
    AERROR << "The smoothed reference line error is too large";
    return false;
  }
  return true;
}
}  // namespace planning
}  // namespace apollo
