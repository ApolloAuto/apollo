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

#include "modules/planning/tasks/pull_over_path/pull_over_path.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/point_factory.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::JunctionInfo;

bool PullOverPath::Init(const std::string& config_dir, const std::string& name,
                        const std::shared_ptr<DependencyInjector>& injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  return Task::LoadConfig<PullOverPathConfig>(&config_);
}

apollo::common::Status PullOverPath::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  if (!reference_line_info->path_data().Empty() ||
      reference_line_info->IsChangeLanePath()) {
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
    ADEBUG << "pull-over path success";
  }

  return Status::OK();
}

bool PullOverPath::DecidePathBounds(std::vector<PathBoundary>* boundary) {
  auto* pull_over_status = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_pull_over();
  const bool plan_pull_over_path = pull_over_status->plan_pull_over_path();
  if (!plan_pull_over_path) {
    return false;
  }
  boundary->emplace_back();
  auto& path_bound = boundary->back();
  double path_narrowest_width = 0;
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_,
                                               &path_bound, init_sl_state_)) {
    AERROR << "Failed to initialize path boundaries.";
    return false;
  }
  if (!GetBoundaryFromRoads(*reference_line_info_, &path_bound)) {
    AERROR << "Failed to decide a rough boundary based on road boundary.";
    return false;
  }
  RecordDebugInfo(path_bound, "pull_over_road", reference_line_info_);
  PathBoundsDeciderUtil::ConvertBoundarySAxisFromLaneCenterToRefLine(
      *reference_line_info_, &path_bound);

  if (init_sl_state_.second[0] < path_bound.front().l_lower.l ||
      init_sl_state_.second[0] > path_bound.front().l_upper.l) {
    AERROR << "ADC is outside road boundary already. Cannot generate pull-over "
              "path";
    return false;
  }
  bool is_pull_over_right = true;
  if (config_.pull_over_direction() == PullOverPathConfig::BOTH_SIDE) {
    double adc_to_left_bound =
        path_bound.front().l_upper.l - init_sl_state_.second[0];
    double adc_to_right_bound =
        init_sl_state_.second[0] - path_bound.front().l_upper.l;
    is_pull_over_right = adc_to_left_bound > adc_to_right_bound;
  } else if (config_.pull_over_direction() == PullOverPathConfig::LEFT_SIDE) {
    is_pull_over_right = false;
  } else {
    is_pull_over_right = true;
  }
  // 2. Update boundary by lane boundary for pull_over
  UpdatePullOverBoundaryByLaneBoundary(is_pull_over_right, &path_bound);
  RecordDebugInfo(path_bound, "pull_over_lane", reference_line_info_);

  std::string blocking_obstacle_id = "";
  PathBound temp_path_bound = path_bound;
  std::vector<SLPolygon> obs_sl_polygons;
  PathBoundsDeciderUtil::GetSLPolygons(*reference_line_info_, &obs_sl_polygons,
                                       init_sl_state_);
  if (!PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
          *reference_line_info_, &obs_sl_polygons, init_sl_state_, &path_bound,
          &blocking_obstacle_id, &path_narrowest_width)) {
    AERROR << "Failed to decide fine tune the boundaries after "
              "taking into consideration all static obstacles.";
    return false;
  }
  // If already found a pull-over position, simply check if it's valid.
  int curr_idx = -1;
  if (pull_over_status->has_position()) {
    curr_idx = PathBoundsDeciderUtil::IsPointWithinPathBound(
        *reference_line_info_, pull_over_status->position().x(),
        pull_over_status->position().y(), path_bound);
  }
  // If haven't found a pull-over position, search for one.
  if (curr_idx < 0) {
    pull_over_status->Clear();
    pull_over_status->set_plan_pull_over_path(true);
    std::tuple<double, double, double, int> pull_over_configuration;
    if (!SearchPullOverPosition(path_bound, &pull_over_configuration)) {
      AERROR << "Failed to find a proper pull-over position.";
      return false;
    }
    const auto& vehicle_param =
        VehicleConfigHelper::GetConfig().vehicle_param();
    curr_idx = std::get<3>(pull_over_configuration);
    // If have found a pull-over position, update planning-context
    pull_over_status->mutable_position()->set_x(
        std::get<0>(pull_over_configuration));
    pull_over_status->mutable_position()->set_y(
        std::get<1>(pull_over_configuration));
    pull_over_status->mutable_position()->set_z(0.0);
    pull_over_status->set_theta(std::get<2>(pull_over_configuration));
    pull_over_status->set_length_front(vehicle_param.front_edge_to_center());
    pull_over_status->set_length_back(vehicle_param.back_edge_to_center());
    pull_over_status->set_width_left(vehicle_param.width() / 2.0);
    pull_over_status->set_width_right(vehicle_param.width() / 2.0);

    AINFO << "Pull Over: x[" << std::fixed << pull_over_status->position().x()
          << "] y[" << pull_over_status->position().y() << "] theta["
          << pull_over_status->theta() << "]";
  }
  // Trim path-bound properly
  while (static_cast<int>(path_bound.size()) - 1 >
         curr_idx + FLAGS_num_extra_tail_bound_point) {
    path_bound.pop_back();
  }
  for (size_t idx = curr_idx + 1; idx < path_bound.size(); ++idx) {
    path_bound[idx].l_lower.l = path_bound[curr_idx].l_lower.l;
    path_bound[idx].l_upper.l = path_bound[curr_idx].l_upper.l;
  }
  return true;
}

bool PullOverPath::OptimizePath(
    const std::vector<PathBoundary>& path_boundaries,
    std::vector<PathData>* candidate_path_data) {
  auto path_config = config_.path_optimizer_config();
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  std::array<double, 3> end_state = {0.0, 0.0, 0.0};

  for (const auto& path_boundary : path_boundaries) {
    size_t path_boundary_size = path_boundary.boundary().size();
    if (path_boundary_size <= 1U) {
      AERROR << "Get invalid path boundary with size: " << path_boundary_size;
      return false;
    }
    std::vector<double> opt_l, opt_dl, opt_ddl;
    std::vector<std::pair<double, double>> ddl_bounds;
    PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line,
                                         &ddl_bounds);
    const auto& pull_over_status =
        injector_->planning_context()->planning_status().pull_over();
    std::vector<double> weight_ref_l(path_boundary_size,
                                     path_config.path_reference_l_weight());
    std::vector<double> ref_l(path_boundary_size, 0);
    if (pull_over_status.has_position() &&
        pull_over_status.position().has_x() &&
        pull_over_status.position().has_y()) {
      common::SLPoint pull_over_sl;
      reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);
      end_state[0] = pull_over_sl.l();
      ref_l.assign(path_boundary_size, end_state[0]);
      weight_ref_l.assign(path_boundary_size, config_.pull_over_weight());
    }

    const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(
        std::fmax(init_sl_state_.first[1], 1e-12));

    bool res_opt = PathOptimizerUtil::OptimizePath(
        init_sl_state_, end_state, ref_l, weight_ref_l, path_boundary,
        ddl_bounds, jerk_bound, path_config, &opt_l, &opt_dl, &opt_ddl);
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

bool PullOverPath::AssessPath(std::vector<PathData>* candidate_path_data,
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
  reference_line_info_->SetBlockingObstacle(
      curr_path_data.blocking_obstacle_id());
  auto* pull_over_debug = reference_line_info_->mutable_debug()
                              ->mutable_planning_data()
                              ->mutable_pull_over();
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  pull_over_debug->mutable_position()->CopyFrom(pull_over_status.position());
  pull_over_debug->set_theta(pull_over_status.theta());
  pull_over_debug->set_length_front(pull_over_status.length_front());
  pull_over_debug->set_length_back(pull_over_status.length_back());
  pull_over_debug->set_width_left(pull_over_status.width_left());
  pull_over_debug->set_width_right(pull_over_status.width_right());
  return true;
}

bool PullOverPath::GetBoundaryFromRoads(
    const ReferenceLineInfo& reference_line_info,
    PathBoundary* const path_bound) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  double adc_lane_width = PathBoundsDeciderUtil::GetADCLaneWidth(
      reference_line, init_sl_state_.first[0]);
  // Go through every point, update the boudnary based on the road boundary.
  double past_road_left_width = adc_lane_width / 2.0;
  double past_road_right_width = adc_lane_width / 2.0;
  int path_blocked_idx = -1;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    // 1. Get road boundary.
    double curr_s = (*path_bound)[i].s;
    double curr_road_left_width = 0.0;
    double curr_road_right_width = 0.0;
    double refline_offset_to_lane_center = 0.0;
    reference_line.GetOffsetToMap(curr_s, &refline_offset_to_lane_center);
    if (!reference_line.GetRoadWidth(curr_s, &curr_road_left_width,
                                     &curr_road_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_road_left_width = past_road_left_width;
      curr_road_right_width = past_road_right_width;
    } else {
      curr_road_left_width += refline_offset_to_lane_center;
      curr_road_right_width -= refline_offset_to_lane_center;
      past_road_left_width = curr_road_left_width;
      past_road_right_width = curr_road_right_width;
    }
    double curr_left_bound = curr_road_left_width;
    double curr_right_bound = -curr_road_right_width;
    ADEBUG << "At s = " << curr_s
           << ", left road bound = " << curr_road_left_width
           << ", right road bound = " << curr_road_right_width
           << ", offset from refline to lane-center = "
           << refline_offset_to_lane_center;

    // 2. Update into path_bound.
    if (!PathBoundsDeciderUtil::UpdatePathBoundaryWithBuffer(
            curr_left_bound, curr_right_bound, BoundType::ROAD, BoundType::ROAD,
            "", "", &path_bound->at(i))) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) {
      break;
    }
  }

  PathBoundsDeciderUtil::TrimPathBounds(path_blocked_idx, path_bound);
  return true;
}

void PullOverPath::UpdatePullOverBoundaryByLaneBoundary(
    bool is_pull_over_right, PathBoundary* const path_bound) {
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  for (size_t i = 0; i < path_bound->size(); ++i) {
    const double curr_s = (*path_bound)[i].s;
    double left_bound = 3.0;
    double right_bound = 3.0;
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    if (reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                    &curr_lane_right_width)) {
      double offset_to_lane_center = 0.0;
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      left_bound = curr_lane_left_width + offset_to_lane_center;
      right_bound = curr_lane_right_width + offset_to_lane_center;
    }
    ADEBUG << "left_bound[" << left_bound << "] right_bound[" << right_bound
           << "]";
    if (is_pull_over_right) {
      (*path_bound)[i].l_upper.l = left_bound;
    } else {
      (*path_bound)[i].l_lower.l = right_bound;
    }
  }
}

bool PullOverPath::SearchPullOverPosition(
    const PathBound& path_bound,
    std::tuple<double, double, double, int>* const pull_over_configuration) {
  // search direction
  bool search_backward = false;  // search FORWARD by default

  double pull_over_s = 0.0;
  if (config_.pull_over_position() == PullOverPathConfig::NEAREST_POSITION) {
    if (!FindNearestPullOverS(&pull_over_s)) {
      AERROR << "Failed to find emergency_pull_over s";
      return false;
    }
    search_backward = false;  // search FORWARD from target position
  } else if (config_.pull_over_position() == PullOverPathConfig::DESTINATION) {
    if (!FindDestinationPullOverS(path_bound, &pull_over_s)) {
      AERROR << "Failed to find pull_over s upon destination arrival";
      return false;
    }
    search_backward = true;  // search BACKWARD from target position
  } else {
    return false;
  }

  int idx = 0;
  if (search_backward) {
    // 1. Locate the first point before destination.
    idx = static_cast<int>(path_bound.size()) - 1;
    while (idx >= 0 && path_bound[idx].s > pull_over_s) {
      --idx;
    }
  } else {
    // 1. Locate the first point after emergency_pull_over s.
    while (idx < static_cast<int>(path_bound.size()) &&
           path_bound[idx].s < pull_over_s) {
      ++idx;
    }
  }
  if (idx < 0 || idx >= static_cast<int>(path_bound.size())) {
    AERROR << "Failed to find path_bound index for pull over s";
    return false;
  }
  constexpr double kPulloverLonSearchCoeff = 1.5;
  constexpr double kPulloverLatSearchCoeff = 1.25;
  // Search for a feasible location for pull-over.
  const double pull_over_space_length =
      kPulloverLonSearchCoeff *
          VehicleConfigHelper::GetConfig().vehicle_param().length() -
      FLAGS_obstacle_lon_start_buffer - FLAGS_obstacle_lon_end_buffer;
  const double pull_over_space_width =
      (kPulloverLatSearchCoeff - 1.0) *
      VehicleConfigHelper::GetConfig().vehicle_param().width();
  const double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;

  // 2. Find a window that is close to road-edge.
  // (not in any intersection)
  bool has_a_feasible_window = false;
  while ((search_backward && idx >= 0 &&
          path_bound[idx].s - path_bound.front().s > pull_over_space_length) ||
         (!search_backward && idx < static_cast<int>(path_bound.size()) &&
          path_bound.back().s - path_bound[idx].s > pull_over_space_length)) {
    int j = idx;
    bool is_feasible_window = true;

    // Check if the point of idx is within intersection.
    double pt_ref_line_s = path_bound[idx].s;
    double pt_ref_line_l = 0.0;
    common::SLPoint pt_sl;
    pt_sl.set_s(pt_ref_line_s);
    pt_sl.set_l(pt_ref_line_l);
    common::math::Vec2d pt_xy;
    reference_line_info_->reference_line().SLToXY(pt_sl, &pt_xy);
    common::PointENU hdmap_point;
    hdmap_point.set_x(pt_xy.x());
    hdmap_point.set_y(pt_xy.y());
    ADEBUG << "Pull-over position might be around (" << pt_xy.x() << ", "
           << pt_xy.y() << ")";
    std::vector<std::shared_ptr<const JunctionInfo>> junctions;
    HDMapUtil::BaseMap().GetJunctions(hdmap_point, 1.0, &junctions);
    if (!junctions.empty()) {
      AWARN << "Point is in PNC-junction.";
      idx = search_backward ? idx - 1 : idx + 1;
      continue;
    }

    while ((search_backward && j >= 0 &&
            path_bound[idx].s - path_bound[j].s < pull_over_space_length) ||
           (!search_backward && j < static_cast<int>(path_bound.size()) &&
            path_bound[j].s - path_bound[idx].s < pull_over_space_length)) {
      double curr_s = path_bound[j].s;
      double curr_right_bound = std::fabs(path_bound[j].l_lower.l);
      double curr_road_left_width = 0;
      double curr_road_right_width = 0;
      reference_line_info_->reference_line().GetRoadWidth(
          curr_s, &curr_road_left_width, &curr_road_right_width);
      ADEBUG << "s[" << curr_s << "] curr_road_left_width["
             << curr_road_left_width << "] curr_road_right_width["
             << curr_road_right_width << "]";
      if (curr_road_right_width - (curr_right_bound + adc_half_width) >
          config_.pull_over_road_edge_buffer()) {
        AERROR << "Not close enough to road-edge. Not feasible for pull-over.";
        is_feasible_window = false;
        break;
      }
      const double right_bound = path_bound[j].l_lower.l;
      const double left_bound = path_bound[j].l_upper.l;
      ADEBUG << "left_bound[" << left_bound << "] right_bound[" << right_bound
             << "]";
      if (left_bound - right_bound < pull_over_space_width) {
        AERROR << "Not wide enough to fit ADC. Not feasible for pull-over.";
        is_feasible_window = false;
        break;
      }

      j = search_backward ? j - 1 : j + 1;
    }
    if (j < 0) {
      return false;
    }
    if (is_feasible_window) {
      has_a_feasible_window = true;
      const auto& reference_line = reference_line_info_->reference_line();
      // estimate pull over point to have the vehicle keep same safety
      // distance to front and back
      const auto& vehicle_param =
          VehicleConfigHelper::GetConfig().vehicle_param();
      const double back_clear_to_total_length_ratio =
          (0.5 * (kPulloverLonSearchCoeff - 1.0) * vehicle_param.length() +
           vehicle_param.back_edge_to_center()) /
          vehicle_param.length() / kPulloverLonSearchCoeff;

      int start_idx = j;
      int end_idx = idx;
      if (!search_backward) {
        start_idx = idx;
        end_idx = j;
      }
      auto pull_over_idx = static_cast<size_t>(
          back_clear_to_total_length_ratio * static_cast<double>(end_idx) +
          (1.0 - back_clear_to_total_length_ratio) *
              static_cast<double>(start_idx));

      const auto& pull_over_point = path_bound[pull_over_idx];
      const double pull_over_s = pull_over_point.s;
      const double pull_over_l =
          pull_over_point.l_lower.l + pull_over_space_width / 2.0;
      common::SLPoint pull_over_sl_point;
      pull_over_sl_point.set_s(pull_over_s);
      pull_over_sl_point.set_l(pull_over_l);

      common::math::Vec2d pull_over_xy_point;
      reference_line.SLToXY(pull_over_sl_point, &pull_over_xy_point);
      const double pull_over_x = pull_over_xy_point.x();
      const double pull_over_y = pull_over_xy_point.y();

      // set the pull over theta to be the nearest lane theta rather than
      // reference line theta in case of reference line theta not aligned with
      // the lane
      const auto& reference_point =
          reference_line.GetReferencePoint(pull_over_s);
      double pull_over_theta = reference_point.heading();
      hdmap::LaneInfoConstPtr lane;
      double s = 0.0;
      double l = 0.0;
      auto point =
          common::util::PointFactory::ToPointENU(pull_over_x, pull_over_y);
      if (HDMapUtil::BaseMap().GetNearestLaneWithHeading(
              point, 5.0, pull_over_theta, M_PI_2, &lane, &s, &l) == 0) {
        pull_over_theta = lane->Heading(s);
      }
      *pull_over_configuration =
          std::make_tuple(pull_over_x, pull_over_y, pull_over_theta,
                          static_cast<int>(pull_over_idx));
      break;
    }

    idx = search_backward ? idx - 1 : idx + 1;
  }

  return has_a_feasible_window;
}

bool PullOverPath::FindNearestPullOverS(double* pull_over_s) {
  const double adc_end_s = reference_line_info_->AdcSlBoundary().end_s();
  const double min_turn_radius = common::VehicleConfigHelper::Instance()
                                     ->GetConfig()
                                     .vehicle_param()
                                     .min_turn_radius();
  const double adjust_factor =
      config_.pull_over_approach_lon_distance_adjust_factor();
  const double pull_over_distance = min_turn_radius * 2 * adjust_factor;
  *pull_over_s = adc_end_s + pull_over_distance;
  return true;
}

bool PullOverPath::FindDestinationPullOverS(const PathBound& path_bound,
                                            double* pull_over_s) {
  // destination_s based on routing_end
  const auto& reference_line = reference_line_info_->reference_line();
  common::SLPoint destination_sl;
  const auto& routing_end = *(frame_->local_view().end_lane_way_point);
  reference_line.XYToSL(routing_end.pose(), &destination_sl);
  const double destination_s = destination_sl.s();
  const double adc_end_s = reference_line_info_->AdcSlBoundary().end_s();

  // Check if destination is some distance away from ADC.
  ADEBUG << "Destination s[" << destination_s << "] adc_end_s[" << adc_end_s
         << "]";
  if (destination_s - adc_end_s <
      config_.pull_over_destination_to_adc_buffer()) {
    AERROR << "Destination is too close to ADC. distance["
           << destination_s - adc_end_s << "]";
    return false;
  }

  // Check if destination is within path-bounds searching scope.
  const double destination_to_pathend_buffer =
      config_.pull_over_destination_to_pathend_buffer();
  if (destination_s + destination_to_pathend_buffer >= path_bound.back().s) {
    AERROR << "Destination is not within path_bounds search scope";
    return false;
  }

  *pull_over_s = destination_s;
  return true;
}

}  // namespace planning
}  // namespace apollo
