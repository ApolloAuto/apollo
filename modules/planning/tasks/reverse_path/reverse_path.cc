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

#include "modules/planning/tasks/reverse_path/reverse_path.h"

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

bool ReversePath::Init(
        const std::string& config_dir,
        const std::string& name,
        const std::shared_ptr<DependencyInjector>& injector) {
    if (!Task::Init(config_dir, name, injector)) {
        AINFO << "Failed to initialize task";
        return false;
    }
    // Load the config this task.
    if (!Task::LoadConfig<ReversePathConfig>(&config_)) {
        AINFO << "Failed to load config";
        return false;
    }
    AINFO << "ReversePath config: " << config_.DebugString();
    return true;
}

apollo::common::Status ReversePath::Process(Frame* frame, ReferenceLineInfo* reference_line_info) {
    GetStartPointSLState();
    if (config_.is_considered_square_boundary()
        && reference_line_info->GetJunction(init_sl_state_.first[0], &junction_overlap_) == 0) {
        AERROR << "not in square junction";
        return Status::OK();
    }
    PathBoundary path_boundary;
    PathData path_data;
    AINFO << init_sl_state_.first[0] << "," << init_sl_state_.first[1] << "," << init_sl_state_.second[0] << ","
          << init_sl_state_.second[1];
    double reference_line_backward_s = reference_line_info->reference_line().GetMapPath().accumulated_s().front();
    static constexpr double kEpsilon = 0.1;
    double reference_line_backward_length = init_sl_state_.first[0] - reference_line_backward_s - kEpsilon;
    if (!DecidePathBounds(&path_boundary, reference_line_backward_length)) {
        AERROR << "Decide path bound failed";
        return Status::OK();
    }
    if (!OptimizePathOsqp(path_boundary, &path_data)) {
        AERROR << "Optimize path failed";
        return Status::OK();
    }
    *reference_line_info->mutable_path_data() = path_data;
    // for (const auto& pt : path_data.discretized_path()) {
    //   AINFO << pt.x() << "," << pt.y() << "," << pt.s() << "," << pt.theta();
    // }
    // for (const auto& pt : path_data.frenet_frame_path()) {
    //   AINFO << pt.s() << "," << pt.l() << "," << pt.dl() << "," << pt.ddl();
    // }
    return Status::OK();
}

bool ReversePath::DecidePathBounds(PathBoundary* boundary, double& reference_line_backward_length) {
    if (!InitPathBoundary(boundary, init_sl_state_, reference_line_backward_length)) {
        const std::string msg = "Failed to initialize path boundaries.";
        AERROR << msg;
        return false;
    }
    if (config_.is_considered_square_boundary()) {
        GetBoundaryFromSquare(*reference_line_info_, boundary, init_sl_state_);
    } else if (config_.is_considered_lane_boundary()) {
        if (!PathBoundsDeciderUtil::GetBoundaryFromSelfLane(*reference_line_info_, init_sl_state_, boundary)) {
            AERROR << "Get boundary from self lane failed";
            return false;
        }
    }

    if (init_sl_state_.second[0] > boundary->at(0).l_upper.l || init_sl_state_.second[0] < boundary->at(0).l_lower.l) {
        AINFO << "init l out of range" << init_sl_state_.second[0] << boundary->at(0).l_lower.l << ","
              << boundary->at(0).l_upper.l;
        if (!PathBoundsDeciderUtil::ExtendBoundaryByADC(*reference_line_info_, init_sl_state_, 0.5, boundary)) {
            AERROR << "Failed to decide a rough boundary based on adc.";
            return false;
        }
        AINFO << "extend boundary by adc";
    }
    boundary->set_label("reverse_path");
    RecordDebugInfo(*boundary, boundary->label(), reference_line_info_);
    return true;
}

bool ReversePath::OptimizePath(PathBoundary* path_boundary, PathData* candidate_path_data) {
    double delta_s = path_boundary->delta_s();
    double start_s = path_boundary->at(0).s;
    size_t path_boundary_size = path_boundary->boundary().size();
    FrenetFramePath frenet_frame_path;
    std::vector<double> opt_l, opt_dl, opt_ddl;
    opt_l.resize(path_boundary_size, 0.0);
    opt_dl.resize(path_boundary_size, init_sl_state_.second[1]);
    opt_ddl.resize(path_boundary_size, 0.0);
    for (size_t i = 0; i < path_boundary_size; i++) {
        double cur_l = init_sl_state_.second[0] + init_sl_state_.second[1] * delta_s * i;
        if (cur_l > path_boundary->at(i).l_upper.l || cur_l < path_boundary->at(i).l_lower.l) {
            AINFO << "trim the path" << cur_l << "," << path_boundary->delta_s() * i;
            break;
        }
        common::FrenetFramePoint frenet_frame_point;
        frenet_frame_point.set_s(delta_s * i + start_s);
        frenet_frame_point.set_l(cur_l);
        frenet_frame_point.set_dl(init_sl_state_.second[1]);
        frenet_frame_point.set_ddl(0);
        frenet_frame_path.push_back(std::move(frenet_frame_point));
    }
    if (path_boundary_size <= 1U) {
        AERROR << "Get invalid path boundary with size: " << path_boundary_size;
        return false;
    }
    candidate_path_data->SetReferenceLine(&reference_line_info_->reference_line());
    candidate_path_data->SetFrenetPath(frenet_frame_path);
    candidate_path_data->set_is_reverse_path(true);
    candidate_path_data->set_path_label("reverse_path");
    return true;
}

bool ReversePath::OptimizePathOsqp(PathBoundary& path_boundary, PathData* candidate_path_data) {
    const auto& config = config_.path_optimizer_config();
    const ReferenceLine& reference_line = reference_line_info_->reference_line();
    std::array<double, 3> end_state = {0.0, 0.0, 0.0};
    size_t path_boundary_size = path_boundary.boundary().size();
    if (path_boundary_size <= 1U) {
        AERROR << "Get invalid path boundary with size: " << path_boundary_size;
        return false;
    }
    std::vector<double> opt_l, opt_dl, opt_ddl;
    std::vector<std::pair<double, double>> ddl_bounds;
    PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line, &ddl_bounds);
    std::for_each(ddl_bounds.begin(), ddl_bounds.end(),
        [](std::pair<double, double>& bound) {
              double temp = bound.first;
              bound.first = -bound.second;
              bound.second = -temp;
        });
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
    PathOptimizerUtil::UpdatePathRefWithBound(path_boundary, config.path_reference_l_weight(), &ref_l, &weight_ref_l);

    path_boundary.set_delta_s(0.1);
    SLState reverse_init_sl_state = init_sl_state_;
    reverse_init_sl_state.second[1] = -reverse_init_sl_state.second[1];
    reverse_init_sl_state.second[2] = -reverse_init_sl_state.second[2];
    bool res_opt = PathOptimizerUtil::OptimizePath(
            reverse_init_sl_state,
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

        double frenet_delta_s = 0.0;
        for (auto& point : frenet_frame_path) {
            frenet_delta_s = point.s() - path_boundary.start_s();
            AINFO << point.dl() << "," << point.ddl();
            point.set_s(path_boundary.start_s() - frenet_delta_s);
            point.set_dl(-point.dl());
            point.set_ddl(-point.ddl());
            AINFO << point.dl() << "," << point.ddl();
        }
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
        path_data.set_is_reverse_path(true);
        *candidate_path_data = std::move(path_data);
    }
    path_boundary.set_delta_s(-0.1);
    return true;
}

bool ReversePath::GetBoundaryFromSquare(
        const ReferenceLineInfo& reference_line_info,
        PathBoundary* const path_boundary,
        const SLState& init_sl_state) {
    const auto& hdmap = hdmap::HDMapUtil::BaseMapPtr();
    const auto& junction = hdmap->GetJunctionById(hdmap::MakeMapId(junction_overlap_.object_id));
    const auto& junction_polygon = junction->polygon();
    const auto& reference_line = reference_line_info.reference_line();
    SLBoundary sl_boundary;
    reference_line.GetSLBoundary(junction_polygon, &sl_boundary);
    SLPolygon sl_polygon(sl_boundary, junction_overlap_.object_id);

    for (size_t i = 0; i < path_boundary->size(); i++) {
        auto& point = path_boundary->at(i);
        if (point.s < sl_polygon.MinS() || point.s > sl_polygon.MaxS()) {
            continue;
        }
        double l_min = sl_polygon.GetRightBoundaryByS(point.s);
        double l_max = sl_polygon.GetLeftBoundaryByS(point.s);
        l_min = std::max<double>(l_min, -config_.max_lateral_distance());
        l_max = std::min<double>(l_max, config_.max_lateral_distance());
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

bool ReversePath::InitPathBoundary(
        PathBoundary* const path_bound,
        SLState init_sl_state,
        double& reference_line_backward_length) {
    // Sanity checks.
    CHECK_NOTNULL(path_bound);
    path_bound->clear();
    path_bound->set_delta_s(-0.1);
    double delta_s = path_bound->delta_s();
    double end_s = init_sl_state.first[0] - std::min(config_.max_s_distance(), reference_line_backward_length);
    for (double curr_s = init_sl_state.first[0]; curr_s > end_s; curr_s += delta_s) {
        path_bound->emplace_back(curr_s, -config_.max_lateral_distance(), config_.max_lateral_distance());
    }
    // Return.
    if (path_bound->empty()) {
        AERROR << "Empty path boundary in InitPathBoundary";
        return false;
    }
    return true;
}

}  // namespace planning
}  // namespace apollo
