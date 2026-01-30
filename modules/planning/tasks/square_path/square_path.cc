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

#include "modules/planning/tasks/square_path/square_path.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/planning_base/common/sl_polygon.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"
namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

bool SquarePath::Init(
        const std::string& config_dir,
        const std::string& name,
        const std::shared_ptr<DependencyInjector>& injector) {
    if (!Task::Init(config_dir, name, injector)) {
        return false;
    }
    // Load the config this task.
    return Task::LoadConfig<SquarePathConfig>(&config_);
}

apollo::common::Status SquarePath::Process(Frame* frame, ReferenceLineInfo* reference_line_info) {
    std::vector<PathBoundary> candidate_path_boundaries;
    std::vector<PathData> candidate_path_data;

    GetStartPointSLState();
    AINFO << init_sl_state_.first[0] << "," << init_sl_state_.first[1] << "," << init_sl_state_.second[0] << ","
          << init_sl_state_.second[1];
    common::TrajectoryPoint planning_start_point = frame_->PlanningStartPoint();
    AINFO << std::fixed << "Plan at the starting point: x = " << planning_start_point.path_point().x()
          << ", y = " << planning_start_point.path_point().y()
          << ", and angle = " << planning_start_point.path_point().theta();
    if (!DecidePathBounds(&candidate_path_boundaries)) {
        AERROR << "Decide path bound failed";
        return Status::OK();
    }
    if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
        AERROR << "Optmize path failed";
        return Status::OK();
    }
    if (!AssessPath(&candidate_path_data, reference_line_info)) {
        AERROR << "Path assessment failed";
    }

    return Status::OK();
}

bool SquarePath::DecidePathBounds(std::vector<PathBoundary>* boundary) {
    boundary->emplace_back();
    auto& path_bound = boundary->back();
    std::string blocking_obstacle_id = "";
    std::string lane_type = "";
    double path_narrowest_width = 0;
    if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_, &path_bound, init_sl_state_)) {
        const std::string msg = "Failed to initialize path boundaries.";
        AERROR << msg;
        return false;
    }
    path_bound.set_label(absl::StrCat("regular/", "square"));
    PathBoundsDeciderUtil::GetBoundaryFromSelfLane(*reference_line_info_, init_sl_state_, &path_bound);

    GetBoundaryFromSquare(*reference_line_info_, &path_bound, init_sl_state_);
    PathBound temp_path_bound = path_bound;
    std::vector<SLPolygon> obs_sl_polygons;
    PathBoundsDeciderUtil::GetSLPolygons(*reference_line_info_, &obs_sl_polygons, init_sl_state_);
    if (!PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
                *reference_line_info_,
                &obs_sl_polygons,
                init_sl_state_,
                &path_bound,
                &blocking_obstacle_id,
                &path_narrowest_width)) {
        const std::string msg
                = "Failed to decide fine tune the boundaries after "
                  "taking into consideration all static obstacles.";
        AERROR << msg;
        return false;
    }

    int counter = 0;
    while (!blocking_obstacle_id.empty() && path_bound.size() < temp_path_bound.size()
           && counter < FLAGS_num_extra_tail_bound_point) {
        path_bound.push_back(temp_path_bound[path_bound.size()]);
        counter++;
    }

    if (init_sl_state_.second[0] > path_bound[0].l_upper.l || init_sl_state_.second[0] < path_bound[0].l_lower.l) {
        AINFO << "init l out of range" << init_sl_state_.second[0] << path_bound[0].l_lower.l << ","
              << path_bound[0].l_upper.l;
        return false;
    }

    path_bound.set_blocking_obstacle_id(blocking_obstacle_id);
    RecordDebugInfo(path_bound, path_bound.label(), reference_line_info_);
    return true;
}

bool SquarePath::OptimizePath(
        const std::vector<PathBoundary>& path_boundaries,
        std::vector<PathData>* candidate_path_data) {
    const auto& config = config_.path_optimizer_config();
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
        PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line, &ddl_bounds);
        PrintCurves print_debug;
        for (size_t i = 0; i < path_boundary_size; ++i) {
            double s = static_cast<double>(i) * path_boundary.delta_s() + path_boundary.start_s();
            double kappa = reference_line.GetNearestReferencePoint(s).kappa();
            print_debug.AddPoint("ref_kappa", s, kappa);
        }
        print_debug.PrintToLog();
        const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(std::fmax(init_sl_state_.first[1], 1e-12));
        std::vector<double> ref_l(path_boundary_size, 0);
        std::vector<double> weight_ref_l(path_boundary_size, 0);
        PathOptimizerUtil::UpdatePathRefWithBound(
                path_boundary, config.path_reference_l_weight(), &ref_l, &weight_ref_l);
        bool res_opt = PathOptimizerUtil::OptimizePath(
                init_sl_state_,
                end_state,
                ref_l,
                weight_ref_l,
                path_boundary,
                ddl_bounds,
                jerk_bound,
                config,
                &opt_l,
                &opt_dl,
                &opt_ddl);
        if (res_opt) {
            auto frenet_frame_path = PathOptimizerUtil::ToPiecewiseJerkPath(
                    opt_l, opt_dl, opt_ddl, path_boundary.delta_s(), path_boundary.start_s());
            PathData path_data;
            path_data.SetReferenceLine(&reference_line);
            path_data.SetFrenetPath(std::move(frenet_frame_path));
            if (FLAGS_use_front_axe_center_in_path_planning) {
                auto discretized_path
                        = DiscretizedPath(PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(path_data));
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

bool SquarePath::AssessPath(std::vector<PathData>* candidate_path_data, ReferenceLineInfo* reference_line_info) {
    PathData& curr_path_data = candidate_path_data->back();
    RecordDebugInfo(curr_path_data, curr_path_data.path_label(), reference_line_info_);
    if (!PathAssessmentDeciderUtil::IsValidRegularPath(*reference_line_info_, curr_path_data)) {
        AINFO << "Lane follow path is invalid";
        return false;
    }
    double max_kappa
            = std::tan(vehicle_param_.max_steer_angle() / vehicle_param_.steer_ratio()) / vehicle_param_.wheel_base();
    AINFO << "max_kappa:" << max_kappa;
    // bool is_valid_path = true;
    for (const auto& path_point : curr_path_data.discretized_path()) {
        AINFO << path_point.x() << "," << path_point.y() << "," << path_point.theta() << "," << path_point.kappa()
              << "," << path_point.dkappa();
        if (std::fabs(path_point.kappa()) > max_kappa) {
            AINFO << "Path kappa is too large" << path_point.kappa();
            // is_valid_path = false;
        }
    }
    // if (!is_valid_path) {
    //   return false;
    // }
    reference_line_info->MutableCandidatePathData()->push_back(curr_path_data);
    if (reference_line_info->mutable_path_data()->Empty()) {
        (*reference_line_info->mutable_path_data()) = std::move(curr_path_data);
    }
    return true;
}

bool SquarePath::GetBoundaryFromSquare(
        const ReferenceLineInfo& reference_line_info,
        PathBoundary* const path_boundary,
        const SLState& init_sl_state) {
    hdmap::PathOverlap junction_overlap;
    if (reference_line_info.GetJunction(init_sl_state.first[0], &junction_overlap) == 0) {
        return false;
    }
    const auto& hdmap = hdmap::HDMapUtil::BaseMapPtr();
    const auto& junction = hdmap->GetJunctionById(hdmap::MakeMapId(junction_overlap.object_id));
    const auto& junction_polygon = junction->polygon();
    const auto& reference_line = reference_line_info.reference_line();
    SLBoundary sl_boundary;
    reference_line.GetSLBoundary(junction_polygon, &sl_boundary);
    SLPolygon sl_polygon(sl_boundary, junction_overlap.object_id);

    for (size_t i = 0; i < path_boundary->size(); i++) {
        auto& point = path_boundary->at(i);
        if (point.s < sl_polygon.MinS() || point.s > sl_polygon.MaxS()) {
            continue;
        }
        double l_min = sl_polygon.GetRightBoundaryByS(point.s);
        double l_max = sl_polygon.GetLeftBoundaryByS(point.s);
        double road_left_width = config_.max_lateral_distance();
        double road_right_width = config_.max_lateral_distance();
        if (config_.enable_road_boundary_constraint()) {
            reference_line.GetRoadWidth(point.s, &road_left_width, &road_right_width);
            road_left_width = std::min(road_left_width, config_.max_lateral_distance());
            road_right_width = std::min(road_right_width, config_.max_lateral_distance());
        }
        l_min = std::max<double>(l_min, -road_right_width);
        l_max = std::min<double>(l_max, road_left_width);
        if (point.l_lower.l > l_min) {
            point.l_lower.l = l_min;
            point.l_lower.type = BoundType::ROAD;
        }
        if (point.l_upper.l < l_max) {
            point.l_upper.l = l_max;
            point.l_upper.type = BoundType::ROAD;
        }
    }
    return true;
}

}  // namespace planning
}  // namespace apollo
