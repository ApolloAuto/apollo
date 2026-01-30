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

#include "modules/planning/tasks/lane_borrow_path_generic/lane_borrow_path_generic.h"

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

bool LaneBorrowPathGeneric::Init(
        const std::string& config_dir,
        const std::string& name,
        const std::shared_ptr<DependencyInjector>& injector) {
    if (!Task::Init(config_dir, name, injector)) {
        return false;
    }
    // Load the config this task.
    return Task::LoadConfig<LaneBorrowPathGenericConfig>(&config_);
}

apollo::common::Status LaneBorrowPathGeneric::Process(Frame* frame, ReferenceLineInfo* reference_line_info) {
    if (!config_.is_allow_lane_borrowing() || reference_line_info->path_reusable()) {
        AINFO << "path reusable" << reference_line_info->path_reusable() << ",skip";
        return Status::OK();
    }
    if (!IsNecessaryToBorrowLane()) {
        decided_side_pass_direction_.clear();
        auto* mutable_path_decider_status
                = injector_->planning_context()->mutable_planning_status()->mutable_path_decider();
        mutable_path_decider_status->set_left_borrow(false);
        mutable_path_decider_status->set_right_borrow(false);
        AINFO << "No need to borrow lane";
        return Status::OK();
    }
    std::vector<PathBoundary> candidate_path_boundaries;
    std::vector<PathData> candidate_path_data;

    AINFO << "LaneBorrowPathGeneric, reference_line_info length: " << reference_line_info->reference_line().Length();

    GetStartPointSLState();
    if (!DecidePathBounds(&candidate_path_boundaries)) {
        AERROR << "Failed to decide path boundaries.";
        return Status::OK();
    }
    if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
        AERROR << "Failed to optimize path.";
        return Status::OK();
    }
    if (!AssessPath(&candidate_path_data, reference_line_info->mutable_path_data())) {
        AERROR << "Path assessment failed";
    }

    return Status::OK();
}

bool LaneBorrowPathGeneric::DecidePathBounds(std::vector<PathBoundary>* boundary) {
    AINFO << "side pass direction size:" << decided_side_pass_direction_.size();
    narrowest_width_.clear();
    for (size_t i = 0; i < decided_side_pass_direction_.size(); i++) {
        boundary->emplace_back();
        auto& path_bound = boundary->back();
        std::string blocking_obstacle_id = "";
        std::string borrow_lane_type = "";
        double path_narrowest_width = 0;
        // 1. Initialize the path boundaries to be an indefinitely large area.
        if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_, &path_bound, init_sl_state_)) {
            const std::string msg = "Failed to initialize path boundaries.";
            AERROR << msg;
            boundary->pop_back();
            continue;
        }
        // 2. Decide a rough boundary based on lane info and ADC's position
        if (!GetBoundaryFromNeighborLane(decided_side_pass_direction_[i], &path_bound, &borrow_lane_type)) {
            AERROR << "Failed to decide a rough boundary based on lane and adc.";
            boundary->pop_back();
            continue;
        }
        // TODO(fengzhiqi): ADC buffer
        if (config_.enable_extend_boundary_by_adc()) {
            if (!PathBoundsDeciderUtil::ExtendBoundaryByADC(*reference_line_info_, init_sl_state_, 0.5, &path_bound)) {
                AERROR << "Failed to decide a rough boundary based on adc.";
                boundary->pop_back();
                continue;
            }
        }

        std::string label;
        if (decided_side_pass_direction_[i] == SidePassDirection::LEFT_BORROW) {
            label = "regular/left" + borrow_lane_type;
        } else {
            label = "regular/right" + borrow_lane_type;
        }
        path_bound.set_label(label);

        // path_bound.DebugString("after_neighbor_lane");
        // 3. Fine-tune the boundary based on static obstacles
        AINFO << "LaneBorrowPathGeneric, decided_side_pass_direction: " << decided_side_pass_direction_[i]
              << ", boundary size: " << path_bound.size() << ", path_bound start end s: [ " << path_bound.front().s
              << ", " << path_bound.back().s << "]";

        PathBound temp_path_bound = path_bound;
        LaneBorrowInfo lane_borrow_direction = decided_side_pass_direction_[i] == SidePassDirection::LEFT_BORROW
                ? LaneBorrowInfo::LEFT_BORROW
                : LaneBorrowInfo::RIGHT_BORROW;
        obs_sl_polygons_.clear();
        GetSLPolygons(&obs_sl_polygons_, lane_borrow_direction);
        PathBoundsDeciderUtil::UpdatePathBoundaryBySLPolygon(
                *reference_line_info_,
                &obs_sl_polygons_,
                init_sl_state_,
                &path_bound,
                &blocking_obstacle_id,
                &path_narrowest_width);
        // path_bound.DebugString("after_obs");

        // 3.2 Update boundary width nudge info
        GetBoundaryFromNudgeDecision(decided_side_pass_direction_[i], &path_bound);
        // path_bound.DebugString("after_undge_info");

        // 3.3 Update boundary width Extra Path Bound
        PathBoundsDeciderUtil::AddExtraPathBound(obs_sl_polygons_, &path_bound, init_sl_state_, &blocking_obstacle_id);

        // 4. Append some extra path bound points to avoid zero-length path data.
        int counter = 0;
        while (!blocking_obstacle_id.empty() && path_bound.size() < temp_path_bound.size()
               && counter < FLAGS_num_extra_tail_bound_point) {
            path_bound.push_back(temp_path_bound[path_bound.size()]);
            counter++;
        }

        // lane_borrow_status update
        auto* lane_borrow_status = injector_->planning_context()->mutable_planning_status()->mutable_lane_borrow();
        if (!blocking_obstacle_id.empty()) {
            double current_time = ::apollo::cyber::Clock::NowInSeconds();
            lane_borrow_status->set_block_obstacle_id(blocking_obstacle_id);
            if (lane_borrow_status->lane_borrow_block()) {
                lane_borrow_status->set_block_duration(
                        lane_borrow_status->block_duration() + current_time
                        - lane_borrow_status->last_block_timestamp());
            } else {
                lane_borrow_status->set_block_duration(0);
                lane_borrow_status->set_lane_borrow_block(true);
            }
            lane_borrow_status->set_last_block_timestamp(current_time);
        } else {
            if (lane_borrow_status->lane_borrow_block()) {
                lane_borrow_status->set_block_duration(0);
                lane_borrow_status->set_lane_borrow_block(false);
                lane_borrow_status->set_last_block_timestamp(0);
            }
        }

        ADEBUG << "Completed generating path boundaries.";

        path_bound.set_blocking_obstacle_id(blocking_obstacle_id);
        RecordDebugInfo(path_bound, path_bound.label(), reference_line_info_);
        narrowest_width_.emplace(label, path_narrowest_width);
    }
    return !boundary->empty();
}

bool LaneBorrowPathGeneric::OptimizePath(
        const std::vector<PathBoundary>& path_boundaries,
        std::vector<PathData>* candidate_path_data) {
    const auto& config = config_.path_optimizer_config();
    const ReferenceLine& reference_line = reference_line_info_->reference_line();
    std::array<double, 3> end_state = {0.0, 0.0, 0.0};

    for (const auto& path_boundary : path_boundaries) {
        std::vector<double> opt_l, opt_dl, opt_ddl;
        std::vector<std::pair<double, double>> ddl_bounds;
        PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line, &ddl_bounds);
        const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(std::fmax(init_sl_state_.first[1], 1e-12));
        std::vector<double> ref_l;
        std::vector<double> weight_ref_l;
        bool is_left_side_pass = path_boundary.label().find("left") != std::string::npos;
        PathOptimizerUtil::UpdatePathRefWithBoundInSidePassDirection(
                path_boundary, config.path_reference_l_weight(), &ref_l, &weight_ref_l, is_left_side_pass);

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
            AINFO << "Lane borrow: path labels [ " << path_boundary.label() << " ]";
            AINFO << "dalta s: " << path_boundary.delta_s() << ", boundary size: " << path_boundary.size()
                  << ", frenet path size: " << candidate_path_data->back().frenet_frame_path().size();
        }
    }
    if (candidate_path_data->empty()) {
        return false;
    }
    return true;
}

bool LaneBorrowPathGeneric::AssessPath(std::vector<PathData>* candidate_path_data, PathData* final_path) {
    std::vector<PathData> valid_path_data;
    for (auto& curr_path_data : *candidate_path_data) {
        if (PathAssessmentDeciderUtil::IsValidRegularPath(*reference_line_info_, curr_path_data)) {
            SetPathInfo(&curr_path_data);
            if (reference_line_info_->SDistanceToDestination() < FLAGS_path_trim_destination_threshold) {
                PathAssessmentDeciderUtil::TrimTailingOutLanePoints(&curr_path_data);
            }
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
    auto* mutable_path_decider_status
            = injector_->planning_context()->mutable_planning_status()->mutable_path_decider();
    const Obstacle* blocking_obstacle = reference_line_info_->path_decision()->obstacles().Find(blocking_obstacle_id_);
    if (valid_path_data.size() > 1) {
        if (ComparePathData(
                    *reference_line_info_,
                    valid_path_data[0],
                    valid_path_data[1],
                    blocking_obstacle,
                    narrowest_width_)) {
            *final_path = valid_path_data[0];
        } else {
            *final_path = valid_path_data[1];
        }
    } else {
        *final_path = valid_path_data[0];
    }
    reference_line_info_->MutableCandidatePathData()->push_back(*final_path);
    *(reference_line_info_->mutable_obs_sl_polygons()) = std::move(obs_sl_polygons_);
    RecordDebugInfo(*final_path, final_path->path_label(), reference_line_info_);
    return true;
}

bool LaneBorrowPathGeneric::GetBoundaryFromNeighborLane(
        const SidePassDirection pass_direction,
        PathBoundary* const path_bound,
        std::string* borrow_lane_type) {
    // Sanity checks.
    CHECK_NOTNULL(path_bound);
    ACHECK(!path_bound->empty());
    const ReferenceLine& reference_line = reference_line_info_->reference_line();
    double adc_lane_width = PathBoundsDeciderUtil::GetADCLaneWidth(reference_line, init_sl_state_.first[0]);
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
        if (!reference_line.GetLaneWidth(curr_s, &curr_lane_left_width, &curr_lane_right_width)) {
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
                            ReferenceLineInfo::LaneType::LeftForward,
                            curr_s,
                            &neighbor_lane_id,
                            &curr_neighbor_lane_width)) {
                    ADEBUG << "Borrow left forward neighbor lane." << neighbor_lane_id.id();
                } else if (reference_line_info_->GetNeighborLaneInfo(
                                   ReferenceLineInfo::LaneType::LeftReverse,
                                   curr_s,
                                   &neighbor_lane_id,
                                   &curr_neighbor_lane_width)) {
                    borrowing_reverse_lane = true;
                    ADEBUG << "Borrow left reverse neighbor lane." << neighbor_lane_id.id();
                }
            } else if (pass_direction == SidePassDirection::RIGHT_BORROW) {
                // Borrowing right neighbor lane.
                if (reference_line_info_->GetNeighborLaneInfo(
                            ReferenceLineInfo::LaneType::RightForward,
                            curr_s,
                            &neighbor_lane_id,
                            &curr_neighbor_lane_width)) {
                    ADEBUG << "Borrow right forward neighbor lane." << neighbor_lane_id.id();
                } else if (reference_line_info_->GetNeighborLaneInfo(
                                   ReferenceLineInfo::LaneType::RightReverse,
                                   curr_s,
                                   &neighbor_lane_id,
                                   &curr_neighbor_lane_width)) {
                    borrowing_reverse_lane = true;
                    ADEBUG << "Borrow right reverse neighbor lane." << neighbor_lane_id.id();
                } else {
                    AINFO << "There is no right neighbor lane.";
                }
            }
        }
        // 3. Calculate the proper boundary based on lane-width, ADC's position,
        //    and ADC's velocity.
        double offset_to_map = 0.0;
        reference_line.GetOffsetToMap(curr_s, &offset_to_map);

        double curr_left_bound_lane = curr_lane_left_width
                + (pass_direction == SidePassDirection::LEFT_BORROW ? curr_neighbor_lane_width : 0.0);

        double curr_right_bound_lane = -curr_lane_right_width
                - (pass_direction == SidePassDirection::RIGHT_BORROW ? curr_neighbor_lane_width : 0.0);
        double curr_left_bound = 0.0;
        double curr_right_bound = 0.0;
        curr_left_bound = curr_left_bound_lane - offset_to_map;
        curr_right_bound = curr_right_bound_lane - offset_to_map;

        // 4. Update the boundary.
        if (!PathBoundsDeciderUtil::UpdatePathBoundaryWithBuffer(
                    curr_left_bound, curr_right_bound, BoundType::LANE, BoundType::LANE, "", "", &path_bound->at(i))) {
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

bool LaneBorrowPathGeneric::GetBoundaryFromNudgeDecision(
        const SidePassDirection pass_direction,
        PathBoundary* const path_bound) {
    const auto& nudge_info = ParkDataCenter::Instance()->current_nudge_info(reference_line_info_->key());
    bool is_use_nudge_key_points = FLAGS_enable_nudge_decider && nudge_info.is_enable()
            && nudge_info.extra_nudge_key_points().size() > 1 && nudge_info.extra_nudge_key_points().at(0).size() > 1;
    if (!is_use_nudge_key_points) {
        AINFO << "GetBoundaryFromNudgeDecision, not use nudge key points";
        return true;
    }

    double start_pt_s, end_pt_s;
    int key_points_index = pass_direction == SidePassDirection::LEFT_BORROW ? 0 : 1;
    start_pt_s = nudge_info.extra_nudge_key_points().at(key_points_index).front().s();
    end_pt_s = nudge_info.extra_nudge_key_points().at(key_points_index).back().s();

    if (pass_direction == SidePassDirection::LEFT_BORROW) {
        for (size_t i = 1; i < path_bound->size(); ++i) {
            auto& left_bound = path_bound->at(i).l_upper;
            auto& right_bound = path_bound->at(i).l_lower;
            double check_s = path_bound->at(i).s;
            if (check_s < start_pt_s) {
                continue;
            } else if (check_s < end_pt_s) {
                double nudge_l = 0;
                if (nudge_info.GetInterpolatedNudgeL(true, check_s, &nudge_l)) {
                    right_bound.l = std::max(std::min(nudge_l, left_bound.l), right_bound.l);
                    path_bound->at(i).is_nudge_bound[RIGHT_INDEX] = true;
                }
            } else {
                break;
            }
        }
    } else {
        for (size_t i = 1; i < path_bound->size(); ++i) {
            auto& left_bound = path_bound->at(i).l_upper;
            auto& right_bound = path_bound->at(i).l_lower;
            double check_s = path_bound->at(i).s;
            if (check_s < start_pt_s) {
                continue;
            } else if (check_s < end_pt_s) {
                double nudge_l = 0;
                if (nudge_info.GetInterpolatedNudgeL(false, check_s, &nudge_l)) {
                    left_bound.l = std::min(std::max(nudge_l, right_bound.l), left_bound.l);
                    path_bound->at(i).is_nudge_bound[LEFT_INDEX] = true;
                }
            } else {
                break;
            }
        }
    }
    return true;
}

void LaneBorrowPathGeneric::UpdateSelfPathInfo() {
    auto cur_path = reference_line_info_->path_data();
    if (!cur_path.Empty() && cur_path.path_label().find("self") != std::string::npos
        && cur_path.blocking_obstacle_id().empty() && IsNudgeFinish()) {
        use_self_lane_ = std::max(use_self_lane_ + 1, 0);
        use_self_lane_ = std::min(use_self_lane_, 10);
    } else {
        use_self_lane_ = std::min(use_self_lane_ - 1, 0);
        use_self_lane_ = std::max(use_self_lane_, -10);
    }
    blocking_obstacle_id_ = cur_path.blocking_obstacle_id();
}

bool LaneBorrowPathGeneric::IsNudgeFinish() {
    const auto& nudge_info = ParkDataCenter::Instance()->current_nudge_info(reference_line_info_->key());
    return !nudge_info.is_enable() || nudge_info.extra_nudge_key_points().size() < 2;
}

bool LaneBorrowPathGeneric::IsNecessaryToBorrowLane() {
    auto* mutable_path_decider_status
            = injector_->planning_context()->mutable_planning_status()->mutable_path_decider();
    UpdateSelfPathInfo();
    is_in_path_lane_borrow_scenario_ = mutable_path_decider_status->is_in_path_lane_borrow_scenario();
    decided_side_pass_direction_.clear();
    if (mutable_path_decider_status->left_borrow()) {
        decided_side_pass_direction_.push_back(SidePassDirection::LEFT_BORROW);
    }
    if (mutable_path_decider_status->right_borrow()) {
        decided_side_pass_direction_.push_back(SidePassDirection::RIGHT_BORROW);
    }
    AINFO << "is_in_path_lane_borrow_scenario_: " << is_in_path_lane_borrow_scenario_
          << ", use_self_lane_: " << use_self_lane_;
    if (is_in_path_lane_borrow_scenario_) {
        // If originally borrowing neighbor lane:
        const Frame* last_frame = injector_->frame_history()->Latest();
        bool last_frame_not_in_lane_borrow = nullptr != last_frame && nullptr != last_frame->DriveReferenceLineInfo()
                && (last_frame->DriveReferenceLineInfo()->path_data().path_label().find("lane_change")
                    != std::string::npos);
        if (use_self_lane_ >= 6 || last_frame_not_in_lane_borrow
            || frame_->reference_line_info().size() != last_frame->reference_line_info().size()) {
            // If have been able to use self-lane for some time, then switch to
            // non-lane-borrowing.
            is_in_path_lane_borrow_scenario_ = false;
            decided_side_pass_direction_.clear();
            AINFO << "Switch from LANE-BORROW path to SELF-LANE path.";
        }
    } else {
        // If originally not borrowing neighbor lane:
        AINFO << "Blocking obstacle ID[" << blocking_obstacle_id_ << "]";
        // ADC requirements check for lane-borrowing:
        // if (!HasSingleReferenceLine(*frame_)) {
        //     AINFO << "Not HasSingleReferenceLine.";
        //     return false;
        // }
        if (reference_line_info_->IsChangeLanePath()) {
            AINFO << "Current referenceline is lane change.";
            return false;
        }
        if (!IsWithinSidePassingSpeedADC(*frame_)) {
            AINFO << "Not IsWithinSidePassingSpeedADC.";
            return false;
        }

        // Obstacle condition check for lane-borrowing:
        if (!IsBlockingObstacleFarFromIntersection(*reference_line_info_, blocking_obstacle_id_)) {
            AINFO << "Not IsBlockingObstacleFarFromIntersection.";
            return false;
        }
        // if (!IsLongTermBlockingObstacle()) {
        //   AINFO << "Not IsLongTermBlockingObstacle.";
        //   return false;
        // }
        if (!IsBlockingObstacleWithinDestination(
                    *reference_line_info_, blocking_obstacle_id_, config_.enable_nudge_destination_threshold())) {
            AINFO << "Not IsBlockingObstacleWithinDestination.";
            return false;
        }
        // if (!IsSidePassableObstacle(*reference_line_info_)) {
        //   AINFO << "Not IsSidePassableObstacle.";
        //   return false;
        // }
        if (!config_.enable_active_trigger() && blocking_obstacle_id_.empty()) {
            AINFO << "Not nudge, no blocking_obstacle_id and not active trigger";
            return false;
        }

        if (!IsEnableNudge(*reference_line_info_)) {
            AINFO << "Not nudge, details in ObstacleNudgeDecider";
            return false;
        }

        // switch to lane-borrowing
        if (decided_side_pass_direction_.empty()) {
            // first time init decided_side_pass_direction
            bool left_borrowable;
            bool right_borrowable;
            CheckLaneBorrow(*reference_line_info_, &left_borrowable, &right_borrowable);
            if (!left_borrowable && !right_borrowable) {
                is_in_path_lane_borrow_scenario_ = false;
                AINFO << "LEFT AND RIGHT LANE CAN NOT BORROW";
                return false;
            } else {
                is_in_path_lane_borrow_scenario_ = true;
                if (left_borrowable) {
                    decided_side_pass_direction_.push_back(SidePassDirection::LEFT_BORROW);
                    mutable_path_decider_status->set_left_borrow(true);
                }
                if (right_borrowable) {
                    decided_side_pass_direction_.push_back(SidePassDirection::RIGHT_BORROW);
                    mutable_path_decider_status->set_right_borrow(true);
                }
            }
        }
        use_self_lane_ = 0;
        for (auto& lane_borrow_dir : decided_side_pass_direction_) {
            AINFO << "lane_borrow_dir: " << lane_borrow_dir;
        }
        AINFO << "Switch from SELF-LANE path to LANE-BORROW path.";
    }
    mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(is_in_path_lane_borrow_scenario_);
    return is_in_path_lane_borrow_scenario_;
}

bool LaneBorrowPathGeneric::HasSingleReferenceLine(const Frame& frame) {
    return frame.reference_line_info().size() == 1;
}

bool LaneBorrowPathGeneric::IsWithinSidePassingSpeedADC(const Frame& frame) {
    return frame.PlanningStartPoint().v() < config_.lane_borrow_max_speed();
}

bool LaneBorrowPathGeneric::IsLongTermBlockingObstacle() {
    if (std::abs(use_self_lane_) >= config_.long_term_blocking_obstacle_cycle_threshold()) {
        ADEBUG << "The blocking obstacle is long-term existing.";
        return true;
    } else {
        ADEBUG << "The blocking obstacle is not long-term existing.";
        return false;
    }
}

bool LaneBorrowPathGeneric::IsSidePassableObstacle(const ReferenceLineInfo& reference_line_info) {
    const auto& path_decider_status = injector_->planning_context()->planning_status().path_decider();
    if (blocking_obstacle_id_.empty()) {
        AINFO << "There is no blocking obstacle.";
        return false;
    }
    const Obstacle* blocking_obstacle = reference_line_info.path_decision().obstacles().Find(blocking_obstacle_id_);
    if (blocking_obstacle == nullptr) {
        AINFO << "Blocking obstacle is no longer there.";
        return false;
    }

    return IsNonmovableObstacle(reference_line_info, *blocking_obstacle);
}

bool LaneBorrowPathGeneric::IsEnableNudge(const ReferenceLineInfo& reference_line_info) {
    if (!FLAGS_enable_nudge_decider) {
        return true;
    }
    // const auto& lane_follow_status =
    //     injector_->planning_context()->planning_status().lane_follow();
    // if (!lane_follow_status.lane_follow_block()) {
    //   AINFO << "LaneBorrowPathGeneric::IsEnableNudge, lane_follow is not block, no
    //   need lane borrow"; return false;
    // }
    const auto& nudge_info = ParkDataCenter::Instance()->current_nudge_info(reference_line_info.key());
    if (nudge_info.is_enable() && nudge_info.extra_nudge_key_points().size() > 1) {
        AINFO << "LaneBorrowPathGeneric::IsEnableNudge, Lane Nudge is enable";
        return true;
    }
    return false;
}

void LaneBorrowPathGeneric::CheckLaneBorrow(
        const ReferenceLineInfo& reference_line_info,
        bool* left_neighbor_lane_borrowable,
        bool* right_neighbor_lane_borrowable) {
    const ReferenceLine& reference_line = reference_line_info.reference_line();

    *left_neighbor_lane_borrowable = true;
    *right_neighbor_lane_borrowable = true;

    static constexpr double kLookforwardDistance = 20.0;
    double check_s = reference_line_info.AdcSlBoundary().end_s();
    double lookforward_distance = std::min(check_s + kLookforwardDistance, reference_line.Length());

    // chenck Nudge length
    const auto& nudge_info = ParkDataCenter::Instance()->current_nudge_info(reference_line_info.key());
    if (FLAGS_enable_nudge_decider && nudge_info.is_enable() && nudge_info.extra_nudge_key_points().size() > 1
        && nudge_info.extra_nudge_key_points().at(0).size() > 1) {
        double nudge_length = nudge_info.extra_nudge_key_points().at(0).back().s();
        lookforward_distance = std::max(std::min(lookforward_distance, nudge_length), check_s + 10.0);
    }

    while (check_s < lookforward_distance) {
        auto ref_point = reference_line.GetNearestReferencePoint(check_s);
        if (ref_point.lane_waypoints().empty()) {
            *left_neighbor_lane_borrowable = false;
            *right_neighbor_lane_borrowable = false;
            return;
        }
        auto ptr_lane_info = reference_line_info.LocateLaneInfo(check_s);
        if (ptr_lane_info->lane().left_neighbor_forward_lane_id().empty()
            && ptr_lane_info->lane().left_neighbor_reverse_lane_id().empty()) {
            *left_neighbor_lane_borrowable = false;
        }
        if (ptr_lane_info->lane().right_neighbor_forward_lane_id().empty()
            && ptr_lane_info->lane().right_neighbor_reverse_lane_id().empty()) {
            *right_neighbor_lane_borrowable = false;
        }
        AINFO << "enable_ignore_boundary_type: " << config_.enable_ignore_boundary_type();
        if (!config_.enable_ignore_boundary_type()) {
            const auto waypoint = ref_point.lane_waypoints().front();
            hdmap::LaneBoundaryType::Type lane_boundary_type = hdmap::LaneBoundaryType::UNKNOWN;

            if (*left_neighbor_lane_borrowable) {
                lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
                if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW
                    || lane_boundary_type == hdmap::LaneBoundaryType::DOUBLE_YELLOW
                    || lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
                    *left_neighbor_lane_borrowable = false;
                }
                AINFO << "s[" << check_s << "] left_lane_boundary_type["
                      << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
            }
            if (*right_neighbor_lane_borrowable) {
                lane_boundary_type = hdmap::RightBoundaryType(waypoint);
                if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW
                    || lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
                    *right_neighbor_lane_borrowable = false;
                }
                AINFO << "s[" << check_s << "] right_neighbor_lane_borrowable["
                      << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
            }
        }
        check_s += 2.0;
    }
}

bool LaneBorrowPathGeneric::CheckLaneBoundaryType(
        const ReferenceLineInfo& reference_line_info,
        const double check_s,
        const SidePassDirection& lane_borrow_info) {
    const ReferenceLine& reference_line = reference_line_info.reference_line();
    auto ref_point = reference_line.GetNearestReferencePoint(check_s);
    if (ref_point.lane_waypoints().empty()) {
        return false;
    }
    if (!config_.enable_ignore_boundary_type()) {
        const auto waypoint = ref_point.lane_waypoints().front();
        hdmap::LaneBoundaryType::Type lane_boundary_type = hdmap::LaneBoundaryType::UNKNOWN;
        if (lane_borrow_info == SidePassDirection::LEFT_BORROW) {
            lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
        } else if (lane_borrow_info == SidePassDirection::RIGHT_BORROW) {
            lane_boundary_type = hdmap::RightBoundaryType(waypoint);
        }
        if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW
            || lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
            return false;
        }
    }
    return true;
}

void LaneBorrowPathGeneric::SetPathInfo(PathData* const path_data) {
    std::vector<PathPointDecision> path_decision;
    PathAssessmentDeciderUtil::InitPathPointDecision(*path_data, PathData::PathPointType::IN_LANE, &path_decision);
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
        double middle_s = (ego_sl_boundary.start_s() + ego_sl_boundary.end_s()) / 2.0;
        if (reference_line_info_->reference_line().GetLaneWidth(middle_s, &lane_left_width, &lane_right_width)) {
            // Rough sl boundary estimate using single point lane width
            double back_to_inlane_extra_buffer = 0.2;
            double in_and_out_lane_hysteresis_buffer = is_prev_point_out_lane ? back_to_inlane_extra_buffer : 0.0;
            // For lane-borrow path, as long as ADC is not on the lane of
            // reference-line, it is out on other lanes. It might even be
            // on reverse lane!
            if (ego_sl_boundary.end_l() > lane_left_width + in_and_out_lane_hysteresis_buffer
                || ego_sl_boundary.start_l() < -lane_right_width - in_and_out_lane_hysteresis_buffer) {
                if (path_data->path_label().find("reverse") != std::string::npos) {
                    std::get<1>((path_decision)[i]) = PathData::PathPointType::OUT_ON_REVERSE_LANE;
                } else if (path_data->path_label().find("forward") != std::string::npos) {
                    std::get<1>((path_decision)[i]) = PathData::PathPointType::OUT_ON_FORWARD_LANE;
                } else {
                    std::get<1>((path_decision)[i]) = PathData::PathPointType::UNKNOWN;
                }
                if (!is_prev_point_out_lane) {
                    if (ego_sl_boundary.end_l() > lane_left_width + back_to_inlane_extra_buffer
                        || ego_sl_boundary.start_l() < -lane_right_width - back_to_inlane_extra_buffer) {
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
            AERROR << "reference line not ready when setting path point guide, middle_s" << middle_s << ",index" << i
                   << "path point" << discrete_path[i].DebugString();
            break;
        }
    }
    path_data->SetPathPointDecisionGuide(std::move(path_decision));
}

void LaneBorrowPathGeneric::GetSLPolygons(std::vector<SLPolygon>* polygons, LaneBorrowInfo lane_borrow_info) {
    polygons->clear();
    auto obstacles = reference_line_info_->path_decision()->obstacles();
    const auto& nudge_info = ParkDataCenter::Instance()->current_nudge_info(reference_line_info_->key());
    const double adc_back_edge_s = reference_line_info_->AdcSlBoundary().start_s();
    for (const auto* obstacle : obstacles.Items()) {
        if (!PathBoundsDeciderUtil::IsWithinPathDeciderScopeObstacle(*obstacle)) {
            continue;
        }
        // double check_dis = std::max(reference_line_info_->vehicle_state().linear_velocity() * 3.0, 10.0);

        // if (nudge_info.IsObsIgnoreNudgeDecision(
        //             obstacle->Id(), obstacle->PerceptionSLBoundary().start_s(), check_dis)) {
        //     continue;
        // }
        if (obstacle->PerceptionSLBoundary().end_s() < adc_back_edge_s) {
            continue;
        }
        const auto obstacle_sl = obstacle->PerceptionSLBoundary();
        // AINFO << "GetSLPolygonsWithNudgeDecision: " << obstacle->Id();
        polygons->emplace_back(obstacle_sl, obstacle->Id(), obstacle->Perception().type());
        if (nudge_info.NeedCheckObsCollision(obstacle->Id())) {
            if (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW) {
                polygons->back().SetNudgeInfo(SLPolygon::LEFT_NUDGE);
                AINFO << "Set Obs " << obstacle->Id() << ", LEFT_NUDGE";
            } else if (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW) {
                polygons->back().SetNudgeInfo(SLPolygon::RIGHT_NUDGE);
                AINFO << "Set Obs " << obstacle->Id() << ", RIGHT_NUDGE";
            }
        }
    }
    sort(polygons->begin(), polygons->end(), [](const SLPolygon& a, const SLPolygon& b) {
        return a.MinS() < b.MinS();
    });
}

bool ComparePathData(
        const ReferenceLineInfo& reference_line_info,
        const PathData& lhs,
        const PathData& rhs,
        const Obstacle* blocking_obstacle,
        const std::unordered_map<std::string, double>& narrowest_width) {
    AINFO << "Comparing " << lhs.path_label() << " and " << rhs.path_label();
    static constexpr double kNeighborPathLengthComparisonTolerance = 25.0;
    double lhs_path_length = lhs.frenet_frame_path().back().s();
    double rhs_path_length = rhs.frenet_frame_path().back().s();
    // Select longer path.
    // If roughly same length, then select self-lane path.
    if (std::fabs(lhs_path_length - rhs_path_length) > kNeighborPathLengthComparisonTolerance) {
        return lhs_path_length > rhs_path_length;
    }
    // If roughly same length, and must borrow neighbor lane,
    // then prefer to borrow forward lane rather than reverse lane.
    int lhs_on_reverse = ContainsOutOnReverseLane(lhs.path_point_decision_guide());
    int rhs_on_reverse = ContainsOutOnReverseLane(rhs.path_point_decision_guide());
    // TODO(jiacheng): make this a flag.
    if (std::abs(lhs_on_reverse - rhs_on_reverse) > 6) {
        AINFO << "ComparePathData, on_reverse: [left, right] : [ " << lhs_on_reverse << ", " << rhs_on_reverse << " ]";
        return lhs_on_reverse < rhs_on_reverse;
    }
    // For two lane-borrow directions, based on ADC's position,
    // select the more convenient one.
    if (blocking_obstacle) {
        // select left/right path based on blocking_obstacle's position
        double obstacle_l = (blocking_obstacle->PerceptionSLBoundary().start_l()
                             + blocking_obstacle->PerceptionSLBoundary().end_l())
                / 2;
        ADEBUG << "ComparePathData, obstacle " << blocking_obstacle->Id() << ", l: " << obstacle_l;
        SLPolygon blocking_obstacle_sl_polygon;
        ParkDataCenter::Instance()
                ->current_nudge_info(reference_line_info.key())
                .GetFirstBlockSLPolygon(blocking_obstacle->Id(), &blocking_obstacle_sl_polygon);
        if (!blocking_obstacle_sl_polygon.id().empty()) {
            obstacle_l = (blocking_obstacle_sl_polygon.MaxL() + blocking_obstacle_sl_polygon.MinL()) / 2;
            AINFO << "ComparePathData, obstacle in nudge_info " << blocking_obstacle_sl_polygon.id()
                  << ", l: " << obstacle_l;
            if (obstacle_l > 0.3) {
                return lhs.path_label().find("right") != std::string::npos;
            } else if (obstacle_l < -0.3) {
                return lhs.path_label().find("left") != std::string::npos;
            }
        }
    } else {
        // select left/right path based on ADC's position
        double adc_l = lhs.frenet_frame_path().front().l();
        AINFO << "ComparePathData, adc_l: " << adc_l;
        if (adc_l < -1.0) {
            AINFO << "adc_l < -1.0";
            return lhs.path_label().find("right") != std::string::npos;
        } else if (adc_l > 1.0) {
            AINFO << "adc_l > 1.0";
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
        AINFO << "ComparePathData, back_idx [ " << lhs_back_idx << ", " << rhs_back_idx << " ], bask_s [ " << lhs_back_s
              << ", " << rhs_back_s << " ]";
        return lhs_back_idx < rhs_back_idx;
    }
    // Compare the narrowest_width
    if (narrowest_width.find(lhs.path_label()) != narrowest_width.end()
        && narrowest_width.find(rhs.path_label()) != narrowest_width.end()) {
        double lhs_w = narrowest_width.at(lhs.path_label());
        double rhs_w = narrowest_width.at(rhs.path_label());
        if ((lhs_w < 2.0 || rhs_w < 2.0) && std::fabs(lhs_w - rhs_w) > 1.5) {
            AINFO << "ComparePathData, lhs_w: " << lhs_w << ", rhs_w: " << rhs_w << ", chose left: " << (lhs_w > rhs_w);
            return lhs_w > rhs_w;
        }
    }
    // If same length, both forward, back to inlane at same time,
    // select the left one to side-pass.
    bool lhs_on_leftlane = lhs.path_label().find("left") != std::string::npos;
    AINFO << "ComparePathData, lhs_on_leftlane: " << lhs_on_leftlane;
    return lhs_on_leftlane;
}

int ContainsOutOnReverseLane(const std::vector<PathPointDecision>& path_point_decision) {
    int ret = 0;
    for (const auto& curr_decision : path_point_decision) {
        if (std::get<1>(curr_decision) == PathData::PathPointType::OUT_ON_REVERSE_LANE) {
            ++ret;
        }
    }
    return ret;
}

int GetBackToInLaneIndex(const std::vector<PathPointDecision>& path_point_decision) {
    // ACHECK(!path_point_decision.empty());
    // ACHECK(std::get<1>(path_point_decision.back()) ==
    //       PathData::PathPointType::IN_LANE);

    for (int i = static_cast<int>(path_point_decision.size()) - 1; i >= 0; --i) {
        if (std::get<1>(path_point_decision[i]) != PathData::PathPointType::IN_LANE) {
            return i;
        }
    }
    return 0;
}

}  // namespace planning
}  // namespace apollo
