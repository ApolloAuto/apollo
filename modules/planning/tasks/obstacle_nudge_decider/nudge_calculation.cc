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

#include "modules/planning/tasks/obstacle_nudge_decider/nudge_calculation.h"

#include <algorithm>
#include <memory>

#include "modules/common_msgs/planning_msgs/decision.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/util.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::common::math::Vec2d;

NudgeCalculation::NudgeCalculation(const ObstacleNudgeDeciderConfig& nudge_config) {
    config_.CopyFrom(nudge_config);
    const auto& vehicle_config = common::VehicleConfigHelper::Instance()->GetConfig();
    veh_config_.width = vehicle_config.vehicle_param().width();
    veh_config_.length = vehicle_config.vehicle_param().length();
    veh_config_.front_edge_to_center = vehicle_config.vehicle_param().front_edge_to_center();
}

void NudgeCalculation::BuildNudgeDecisionWithObs(
        Frame* const current_frame,
        const Frame* last_frame,
        ReferenceLineInfo* const reference_line_info) {
    auto* nudge_info = ParkDataCenter::Instance()->mutable_current_nudge_info(reference_line_info->key());
    nudge_info->mutable_update_ids()->clear();
    nudge_info->mutable_block_sl_polygons()->clear();
    adc_boundary_ = reference_line_info->AdcSlBoundary();
    auto* tracking_nudge_obs_info = nudge_info->mutable_tracking_nudge_obs_info();

    double ego_v = std::abs(reference_line_info->vehicle_state().linear_velocity());
    check_forward_dis_ = common::math::Clamp(ego_v, 0.0, 8.0) / 8.0
                    * (config_.max_forward_check_dis() - config_.min_forward_check_dis())
            + config_.min_forward_check_dis();

    dis2end_ = reference_line_info->SDistanceToDestination();

    if (nullptr != last_frame && nullptr != last_frame->DriveReferenceLineInfo()
        && ParkDataCenter::Instance()->GetLastFrameNudgeTrackInfo(
                last_frame->SequenceNum(), last_frame->DriveReferenceLineInfo()->key(), tracking_nudge_obs_info)) {
        AINFO << "NudgeCalculation: update tracking_nudge_obs_info from last frame";
    }

    for (auto& iter : nudge_info->tracking_nudge_obs_info()) {
        AINFO << "last frame tracking info: " << iter.first << ", P(nudge): " << iter.second.nudge_probability;
    }

    for (const auto* obstacle : reference_line_info->path_decision()->obstacles().Items()) {
        const std::string& obstacle_id = obstacle->Id();
        const std::string obstacle_type_name = PerceptionObstacle_Type_Name(obstacle->Perception().type());

        AINFO << "NudgeCalculation: check obstacle id [ " << obstacle_id << " ], type [ " << obstacle_type_name << " ]";

        if (!IsWithinNudgeScopeObstacle(reference_line_info, *obstacle)) {
            AINFO << "NudgeCalculation: Not IsWithinNudgeScopeObstacle, Skip";
            continue;
        }

        // update_static_tracking_obs
        if (obstacle->IsStatic()) {
            BuildStaticTrackingObs(reference_line_info, *obstacle, nudge_info);
        } else {
            // To Do
            ADEBUG << "NudgeCalculation: obstacle [ " << obstacle->Id() << "] is dynamic";
        }
    }

    // clear tracking obs which is not updated
    for (auto iter = tracking_nudge_obs_info->begin(); iter != tracking_nudge_obs_info->end();) {
        auto obs_id_iter = nudge_info->update_ids().find(iter->first);
        if (obs_id_iter != nudge_info->update_ids().end()) {
            ++iter;
            continue;
        }

        double current_time = ::apollo::cyber::Clock::NowInSeconds();
        if (current_time - iter->second.last_track_time > config_.obs_clean_time_out_threshold()) {
            AINFO << "NudgeCalculation: erase tracking_nudge_obs_info: obs [ " << iter->first << " ]";
            iter = tracking_nudge_obs_info->erase(iter);
            continue;
        }
        ++iter;
    }

    // update nudge
    for (const auto& obs_id : nudge_info->update_ids()) {
        auto track_obs_iter = tracking_nudge_obs_info->find(obs_id);
        if (track_obs_iter != tracking_nudge_obs_info->end()) {
            nudge_info->mutable_block_sl_polygons()->emplace_back(track_obs_iter->second.sl_boundary, obs_id);
            // track_obs_iter->second.PrintPolygonInfo(obs_id);
            AINFO << "update nudge [ " << obs_id << " ], "
                  << "nudge_probability: " << track_obs_iter->second.nudge_probability;
        }
    }

    // sort sl polygons
    nudge_info->SortBlockSLPolygons();

    // merge first block obs width nearby obstacle
    bool is_merge_obs_polygon = MergeObsPolygon(reference_line_info, nudge_info);
    nudge_info->set_is_merge_block_obs(is_merge_obs_polygon);
    AINFO << "NudgeCalculation: is_merge_obs_polygon: " << is_merge_obs_polygon;
    AINFO << "block_sl_polygons size : " << nudge_info->block_sl_polygons().size();

    // calculate nudge_probability to make nudge decision
    if (IsProbEnoughToNudge(nudge_info, is_merge_obs_polygon)) {
        CalcNudgeExtraSpace(reference_line_info, nudge_info);
        nudge_info->set_enable(true);
        AINFO << "NudgeCalculation: Enable Nudge";
    } else {
        nudge_info->set_enable(false);
        AINFO << "NudgeCalculation: Not Enable Nudge";
    }
    // TODO(fengzhiqi): nudge safety check
}

void NudgeCalculation::BuildStaticTrackingObs(
        ReferenceLineInfo* const reference_line_info,
        const Obstacle& obstacle,
        NudgeInfo* const nudge_info) {
    std::string obs_id = obstacle.Id();
    RemainNudgeSpace remain_nudge_space;
    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    SLBoundary obs_boundary = obstacle.PerceptionSLBoundary();
    GetRefLineLaneWidth(
            reference_line_info->reference_line(),
            adc_boundary_.start_s(),
            0.5 * (obs_boundary.start_s() + obs_boundary.end_s()),
            &lane_left_width,
            &lane_right_width);
    bool obs_is_all_on_lane = obs_boundary.start_l() > -lane_right_width && obs_boundary.end_l() < lane_left_width;
    bool obs_far_from_lane_boundary = obs_boundary.start_l() - lane_left_width > veh_config_.width
            || obs_boundary.end_l() + lane_right_width < -veh_config_.width;

    bool is_obs_passable = CalcObsRemainSpace(reference_line_info, obstacle, &remain_nudge_space);
    AINFO << "NudgeCalculation: BuildStaticTrackingObs: [ " << obs_id << " ], is passable: " << is_obs_passable
          << ", obs_is_all_on_lane: " << obs_is_all_on_lane;
    auto* tracking_nudge_obs_info = nudge_info->mutable_tracking_nudge_obs_info();
    auto iter = tracking_nudge_obs_info->find(obs_id);
    if (iter != tracking_nudge_obs_info->end()) {
        AINFO << "NudgeCalculation: MatchTrackingObsInfo: [ " << obs_id << " ] with last tracking obs [ " << iter->first
              << " ]";
        UpdateTrackingObsInfo(&iter->second, obstacle, is_obs_passable, remain_nudge_space, obs_is_all_on_lane);
        nudge_info->mutable_update_ids()->insert(obs_id);
    } else {
        // Match obs and update obs info
        if (MatchTrackingObsInfo(obstacle, nudge_info, is_obs_passable, remain_nudge_space, obs_is_all_on_lane)) {
            return;
        }
        if (!is_obs_passable
            || ((obs_is_all_on_lane || !obs_far_from_lane_boundary)
                && (obstacle.Perception().type() == apollo::perception::PerceptionObstacle::UNKNOWN_UNMOVABLE
                    || obstacle.Perception().type() == apollo::perception::PerceptionObstacle::UNKNOWN))) {
            NudgeObstacleInfo new_track_obs_info;
            new_track_obs_info.start_track_time = ::apollo::cyber::Clock::NowInSeconds();
            UpdateTrackingObsInfo(
                    &new_track_obs_info, obstacle, is_obs_passable, remain_nudge_space, obs_is_all_on_lane);
            nudge_info->mutable_update_ids()->insert(obs_id);
            tracking_nudge_obs_info->emplace(obs_id, new_track_obs_info);
        }
    }
}

bool NudgeCalculation::MatchTrackingObsInfo(
        const Obstacle& obstacle,
        NudgeInfo* const nudge_info,
        bool is_passable,
        RemainNudgeSpace remain_nudge_space,
        bool is_all_on_lane) {
    std::string obs_id = obstacle.Id();
    for (auto iter = nudge_info->mutable_tracking_nudge_obs_info()->begin();
         iter != nudge_info->mutable_tracking_nudge_obs_info()->end();
         iter++) {
        // skip the tracking obs which is updated
        auto obs_id_iter = nudge_info->update_ids().find(iter->first);
        if (obs_id_iter != nudge_info->update_ids().end()) {
            continue;
        }
        // polygon match
        // case 1. tracking obs polygon contains obstacle // case 2. polygon IoU >
        // 0.6
        if (iter->second.expand_polygon.Contains(obstacle.PerceptionPolygon())
            || iter->second.expand_polygon.ComputeIoU(obstacle.PerceptionPolygon()) > 0.6) {
            NudgeObstacleInfo new_obs_info = iter->second;
            UpdateTrackingObsInfo(&new_obs_info, obstacle, is_passable, remain_nudge_space, is_all_on_lane);
            AINFO << "NudgeCalculation: MatchTrackingObsInfo [ " << obs_id << " ] with last tracking obs [ "
                  << iter->first << " ]";
            iter = nudge_info->mutable_tracking_nudge_obs_info()->erase(iter);
            nudge_info->mutable_tracking_nudge_obs_info()->emplace(obs_id, new_obs_info);
            nudge_info->mutable_update_ids()->insert(obs_id);
            return true;
        }
    }
    AINFO << "NudgeCalculation: MatchTrackingObsInfo, can not match any obs for [ " << obs_id << " ]";
    return false;
}

bool NudgeCalculation::CalcObsRemainSpace(
        ReferenceLineInfo* const reference_line_info,
        const Obstacle& obstacle,
        RemainNudgeSpace* remain_space) {
    SLBoundary adc_boundary = reference_line_info->AdcSlBoundary();
    SLBoundary obs_boundary = obstacle.PerceptionSLBoundary();
    bool is_left = false;
    double ego_whole_pass_width = 0.5 * veh_config_.width
            + util::CalculateEquivalentEgoWidth(*reference_line_info, obs_boundary.start_s(), &is_left);

    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    double road_left_width = 0.0;
    double road_right_width = 0.0;
    hdmap::LaneBoundaryType::Type left_boundary_type = hdmap::LaneBoundaryType::UNKNOWN;
    hdmap::LaneBoundaryType::Type right_boundary_type = hdmap::LaneBoundaryType::UNKNOWN;

    reference_line_info->reference_line().GetLaneWidth(obs_boundary.start_s(), &lane_left_width, &lane_right_width);
    reference_line_info->reference_line().GetRoadWidth(obs_boundary.start_s(), &road_left_width, &road_right_width);
    reference_line_info->reference_line().GetLaneBoundaryType(
            obs_boundary.start_s(), &left_boundary_type, &right_boundary_type);
    remain_space->left_lane_boundary_type = left_boundary_type;
    remain_space->right_lane_boundary_type = right_boundary_type;

    remain_space->left_lane_remain
            = lane_left_width - ego_whole_pass_width - obs_boundary.end_l() - FLAGS_static_obstacle_nudge_l_buffer;
    remain_space->right_lane_remain
            = lane_right_width - ego_whole_pass_width + obs_boundary.start_l() - FLAGS_static_obstacle_nudge_l_buffer;

    if (remain_space->left_lane_boundary_type == hdmap::LaneBoundaryType::CURB) {
        remain_space->left_road_remain = remain_space->left_lane_remain;
    } else {
        remain_space->left_road_remain
                = road_left_width - ego_whole_pass_width - obs_boundary.end_l() - FLAGS_static_obstacle_nudge_l_buffer;
    }

    if (remain_space->right_lane_boundary_type == hdmap::LaneBoundaryType::CURB) {
        remain_space->right_road_remain = remain_space->right_lane_remain;
    } else {
        remain_space->right_road_remain = road_right_width - ego_whole_pass_width + obs_boundary.start_l()
                - FLAGS_static_obstacle_nudge_l_buffer;
    }

    bool is_passable = remain_space->left_lane_remain > config_.obs_passable_buffer()
            || remain_space->right_lane_remain > config_.obs_passable_buffer();

    ADEBUG << "CalcObsRemainSpace: [ " << obstacle.Id() << " ]" << ", left right lane_remain: [ "
           << remain_space->left_lane_remain << ", " << remain_space->right_lane_remain << " ]"
           << ", lane left right width: " << lane_left_width << ", " << lane_right_width << " ]"
           << ", ego_whole_pass_width: " << ego_whole_pass_width << ", obs_boundary l: [ " << obs_boundary.start_l()
           << ", " << obs_boundary.end_l() << " ]";

    // TODO(fengzhiqi) special buffer for different type obstacles

    return is_passable;
}

void NudgeCalculation::set_in_nudge_state(bool state) {
    is_in_nudge_state_ = state;
}

bool NudgeCalculation::IsWithinNudgeScopeObstacle(
        ReferenceLineInfo* const reference_line_info,
        const Obstacle& obstacle) {
    // Obstacle should be non-virtual.
    if (obstacle.IsVirtual()) {
        ADEBUG << "obstacle IsVirtual";
        return false;
    }
    // Obstacle should not have ignore decision.
    if (obstacle.HasLongitudinalDecision() && obstacle.HasLateralDecision() && obstacle.IsIgnore()) {
        ADEBUG << "obstacle IsIgnore";
        return false;
    }
    // Obstacle should not be moving obstacle.
    if (!obstacle.IsStatic() || obstacle.speed() > FLAGS_static_obstacle_speed_threshold) {
        ADEBUG << "obstacle Is not Static";
        return false;
    }
    // Obstacle should in ROI
    if (obstacle.PerceptionSLBoundary().start_s() - adc_boundary_.end_s() > dis2end_
        || obstacle.PerceptionSLBoundary().start_s() - adc_boundary_.end_s() > check_forward_dis_
        || obstacle.PerceptionSLBoundary().end_s() - adc_boundary_.start_s() < 0) {
        ADEBUG << "obstacle not in ROI";
        return false;
    }

    // check obstacle is in the lane
    bool obs_on_lane = reference_line_info->reference_line().IsOnLane(obstacle.PerceptionSLBoundary());
    if (!obs_on_lane) {
        ADEBUG << "obstacle is out of lane width";
        return false;
    }
    return true;
}

void NudgeCalculation::UpdateTrackingObsInfo(
        NudgeObstacleInfo* const track_obs_info,
        const Obstacle& obstacle,
        bool is_passable,
        RemainNudgeSpace remain_nudge_space,
        bool is_all_on_lane) {
    AINFO << "NudgeCalculation: UpdateTrackingObsInfo: [ " << obstacle.Id() << " ], is_passable: " << is_passable;
    track_obs_info->origin_polygon = obstacle.PerceptionPolygon();
    track_obs_info->expand_polygon = track_obs_info->origin_polygon.ExpandByDistance(config_.expand_polygon_buffer());
    track_obs_info->sl_boundary = obstacle.PerceptionSLBoundary();
    track_obs_info->last_track_time = ::apollo::cyber::Clock::NowInSeconds();
    track_obs_info->is_passable = is_passable;
    track_obs_info->remain_nudge_space = remain_nudge_space;
    track_obs_info->is_all_on_lane = is_all_on_lane;
    if (!track_obs_info->is_passable || track_obs_info->is_all_on_lane) {
        track_obs_info->nudge_probability += 0.08 / config_.obs_nudge_time_out_threshold();
        track_obs_info->nudge_probability = common::math::Clamp(track_obs_info->nudge_probability, 0.0, 1.0);
    }
}

void NudgeCalculation::GetRefLineLaneWidth(
        const ReferenceLine& reference_line,
        const double adc_s,
        const double search_s,
        double* const lane_left_width,
        double* const lane_right_width) {
    double adc_lane_width = PathBoundsDeciderUtil::GetADCLaneWidth(reference_line, adc_s);
    double refline_offset_to_lane_center = 0.0;
    reference_line.GetOffsetToMap(search_s, &refline_offset_to_lane_center);
    if (!reference_line.GetLaneWidth(search_s, lane_left_width, lane_right_width)) {
        *lane_left_width = 0.5 * adc_lane_width - refline_offset_to_lane_center;
        *lane_right_width = 0.5 * adc_lane_width + refline_offset_to_lane_center;
    }
}

bool NudgeCalculation::MergeObsPolygon(ReferenceLineInfo* const reference_line_info, NudgeInfo* const nudge_info) {
    if (nudge_info->block_sl_polygons().size() < 1) {
        AINFO << "MergeObsPolygon: block_sl_polygons is empty, return";
        return false;
    }

    // Find first polygon all on road
    SLPolygon first_sl_polygon;
    for (size_t i = 0; i < nudge_info->block_sl_polygons().size(); ++i) {
        auto iter = nudge_info->tracking_nudge_obs_info().find(nudge_info->block_sl_polygons().at(i).id());
        if (iter != nudge_info->tracking_nudge_obs_info().end() && iter->second.is_all_on_lane) {
            first_sl_polygon = nudge_info->block_sl_polygons().at(0);
            break;
        }
    }
    if (first_sl_polygon.id().empty()) {
        AINFO << "MergeObsPolygon: not found first_sl_polygon";
        for (auto sl_polygon_iter = nudge_info->mutable_block_sl_polygons()->begin();
             sl_polygon_iter != nudge_info->mutable_block_sl_polygons()->end();) {
            auto track_iter = nudge_info->tracking_nudge_obs_info().find((*sl_polygon_iter).id());
            if (track_iter == nudge_info->tracking_nudge_obs_info().end()) {
                break;
            }
            if (track_iter->second.is_passable && !track_iter->second.is_all_on_lane) {
                EraseObsInNudgeinfo(nudge_info, (*sl_polygon_iter).id());
                sl_polygon_iter = nudge_info->mutable_block_sl_polygons()->erase(sl_polygon_iter);
            } else {
                ++sl_polygon_iter;
            }
        }
        return false;
    }
    // Merge from first polygon
    std::string merge_polygon_id = "";
    std::vector<Vec2d> merge_points;
    for (auto sl_polygon_iter = nudge_info->mutable_block_sl_polygons()->begin();
         sl_polygon_iter != nudge_info->mutable_block_sl_polygons()->end();) {
        if (sl_polygon_iter->MinS() - first_sl_polygon.MaxS() > 0.8) {
            break;
        }
        auto track_iter = nudge_info->tracking_nudge_obs_info().find((*sl_polygon_iter).id());
        if (track_iter == nudge_info->tracking_nudge_obs_info().end()) {
            break;
        }
        if (!track_iter->second.is_all_on_lane) {
            ++sl_polygon_iter;
            continue;
        }
        merge_points.insert(
                merge_points.end(),
                track_iter->second.origin_polygon.points().begin(),
                track_iter->second.origin_polygon.points().end());
        merge_polygon_id += (*sl_polygon_iter).id() + "+";
        // erase obs which is merged
        sl_polygon_iter = nudge_info->mutable_block_sl_polygons()->erase(sl_polygon_iter);
    }

    if (merge_points.empty()) {
        return false;
    }
    Polygon2d merge_polygon;
    if (!common::math::Polygon2d::ComputeConvexHull(merge_points, &merge_polygon)) {
        return false;
    }

    // Merge the obstacle near lane boundary
    for (auto sl_polygon_iter = nudge_info->mutable_block_sl_polygons()->begin();
         sl_polygon_iter != nudge_info->mutable_block_sl_polygons()->end();) {
        auto track_iter = nudge_info->tracking_nudge_obs_info().find((*sl_polygon_iter).id());
        if (track_iter == nudge_info->tracking_nudge_obs_info().end()) {
            break;
        }
        if (!track_iter->second.is_passable) {
            ++sl_polygon_iter;
            continue;
        } else if (sl_polygon_iter->MinS() - first_sl_polygon.MaxS() > 0.8) {
            AINFO << "MergeObsPolygon: erase obs far and passable: " << (*sl_polygon_iter).id();
            EraseObsInNudgeinfo(nudge_info, (*sl_polygon_iter).id());
            sl_polygon_iter = nudge_info->mutable_block_sl_polygons()->erase(sl_polygon_iter);
            continue;
        }

        if (!track_iter->second.is_all_on_lane
            && merge_polygon.DistanceTo(track_iter->second.origin_polygon)
                    < veh_config_.width + FLAGS_static_obstacle_nudge_l_buffer * 2.0) {
            merge_points.insert(
                    merge_points.end(),
                    track_iter->second.origin_polygon.points().begin(),
                    track_iter->second.origin_polygon.points().end());
            merge_polygon_id += (*sl_polygon_iter).id() + "+";
        } else {
            // erase obs far from lane boundary
            AINFO << "MergeObsPolygon: erase obs far from lane boundary: " << (*sl_polygon_iter).id();
            EraseObsInNudgeinfo(nudge_info, (*sl_polygon_iter).id());
        }
        // erase obs which is merged
        sl_polygon_iter = nudge_info->mutable_block_sl_polygons()->erase(sl_polygon_iter);
    }
    Polygon2d merge_polygon_update;
    if (!common::math::Polygon2d::ComputeConvexHull(merge_points, &merge_polygon_update)) {
        return false;
    }

    SLBoundary merge_sl_boundary;
    if (!reference_line_info->reference_line().GetSLBoundary(merge_polygon_update, &merge_sl_boundary)) {
        nudge_info->mutable_block_sl_polygons()->insert(
                nudge_info->mutable_block_sl_polygons()->begin(), first_sl_polygon);
        AINFO << "MergeObsPolygon: GetSLBoundary failed";
        return false;
    }

    SLPolygon merge_sl_polygon(merge_sl_boundary, merge_polygon_id);
    nudge_info->mutable_block_sl_polygons()->insert(nudge_info->mutable_block_sl_polygons()->begin(), merge_sl_polygon);

    if (merge_polygon_update.points().size() > 0) {
        PrintCurves print_curve;
        for (const auto& p : merge_polygon_update.points()) {
            print_curve.AddPoint(merge_polygon_id + "_BlockObsPolygon", p.x(), p.y());
        }
        print_curve.AddPoint(
                merge_polygon_id + "_BlockObsPolygon", merge_polygon.points()[0].x(), merge_polygon.points()[0].y());
        print_curve.PrintToLog();
    }
    AINFO << "MergeObsPolygon: " << merge_polygon_id;

    // sort sl polygons again
    nudge_info->SortBlockSLPolygons();
    if (nudge_info->block_sl_polygons().at(0).id() == merge_polygon_id) {
        return true;
    }
    return false;
}

void NudgeCalculation::EraseObsInNudgeinfo(NudgeInfo* const nudge_info, std::string id) {
    nudge_info->mutable_update_ids()->erase(id);
    nudge_info->mutable_tracking_nudge_obs_info()->erase(id);
}

bool NudgeCalculation::IsProbEnoughToNudge(NudgeInfo* const nudge_info, bool is_merge_obs) {
    if (nudge_info->block_sl_polygons().empty()) {
        return false;
    }
    SLPolygon first_block_polygon = nudge_info->block_sl_polygons().at(0);
    double max_nudge_probability = 0.0;
    if (is_merge_obs) {
        std::vector<std::string> block_ids;
        std::string id;
        std::istringstream ids_stream(first_block_polygon.id());
        while (std::getline(ids_stream, id, '+')) {
            block_ids.push_back(id);
        }
        for (const auto& obs_id : block_ids) {
            auto track_iter = nudge_info->tracking_nudge_obs_info().find(obs_id);
            if (track_iter == nudge_info->tracking_nudge_obs_info().end()) {
                continue;
            }
            max_nudge_probability = std::max(track_iter->second.nudge_probability, max_nudge_probability);
        }
    } else {
        auto track_iter = nudge_info->tracking_nudge_obs_info().find(first_block_polygon.id());
        if (track_iter != nudge_info->tracking_nudge_obs_info().end()) {
            max_nudge_probability = std::max(track_iter->second.nudge_probability, max_nudge_probability);
        }
    }
    AINFO << "max_nudge_probability: " << max_nudge_probability;
    return max_nudge_probability > 0.8;
}

void NudgeCalculation::CalcNudgeExtraSpace(ReferenceLineInfo* const reference_line_info, NudgeInfo* const nudge_info) {
    nudge_info->mutable_extra_nudge_key_points()->clear();
    if (nudge_info->block_sl_polygons().size() < 1) {
        AINFO << "No block_sl_polygons, return";
        return;
    }
    double start_s = 1000.0;
    double end_s = -1000.0;
    double ego_whole_pass_width = veh_config_.width;
    // need update
    double extend_length = veh_config_.length * config_.nudge_key_point_extend_length_coefficient();

    int box_index = 0;
    bool is_left = false;
    for (auto& sl_polygon : nudge_info->block_sl_polygons()) {
        if (box_index > 0 && (sl_polygon.MinS() - end_s) > 3.0 * veh_config_.length) {
            break;
        }
        if (box_index < 3) {
            ego_whole_pass_width = 0.5 * veh_config_.width
                    + util::CalculateEquivalentEgoWidth(*reference_line_info, sl_polygon.MinS(), &is_left);
            ++box_index;
        }
        start_s = std::min(sl_polygon.MinS() - extend_length, start_s);
        end_s = std::max(sl_polygon.MaxS() + extend_length, end_s);
    }
    start_s = std::max(adc_boundary_.start_s(), start_s);
    end_s = std::max(adc_boundary_.start_s(), end_s);

    std::vector<SLPoint> left_nudge_pts, right_nudge_pts;
    for (double s = start_s; s <= end_s; s += 0.1) {
        double left_max_nudge_l = -1000.0;
        double right_min_nudge_l = 1000.0;
        for (size_t index = 0; index < nudge_info->block_sl_polygons().size(); ++index) {
            SLPolygon current_polygon = nudge_info->block_sl_polygons().at(index);
            if ((index > 0 && s < current_polygon.MaxS()) || end_s < current_polygon.MinS()) {
                continue;
            }
            // if ((index < nudge_info->block_sl_polygons().size() - 1 && s > current_polygon.MaxS())
            //     || end_s < current_polygon.MinS()) {
            //     continue;
            // }
            left_max_nudge_l = std::max(
                    current_polygon.MaxL() + 0.5 * ego_whole_pass_width + FLAGS_static_obstacle_nudge_l_buffer,
                    left_max_nudge_l);
            right_min_nudge_l = std::min(
                    current_polygon.MinL() - 0.5 * ego_whole_pass_width - FLAGS_static_obstacle_nudge_l_buffer,
                    right_min_nudge_l);
            if (s < current_polygon.MinS()) {
                break;
            }
        }
        SLPoint left_pt, right_pt;

        left_pt.set_s(s);
        left_pt.set_l(left_max_nudge_l);
        right_pt.set_s(s);
        right_pt.set_l(right_min_nudge_l);
        left_nudge_pts.emplace_back(left_pt);
        right_nudge_pts.emplace_back(right_pt);
    }
    nudge_info->mutable_extra_nudge_key_points()->emplace_back(left_nudge_pts);
    nudge_info->mutable_extra_nudge_key_points()->emplace_back(right_nudge_pts);

    PrintCurves left_print_curve;
    for (auto& pt : nudge_info->extra_nudge_key_points().at(0)) {
        left_print_curve.AddPoint("LeftKeyPoints_NudgePoints", pt.s(), pt.l());
    }
    left_print_curve.PrintToLog();

    PrintCurves right_print_curve;
    for (auto& pt : nudge_info->extra_nudge_key_points().at(1)) {
        right_print_curve.AddPoint("RightKeyPoints_NudgePoints", pt.s(), pt.l());
    }
    right_print_curve.PrintToLog();
}

}  // namespace planning
}  // namespace apollo
