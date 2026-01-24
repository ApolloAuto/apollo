/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/park_data_center/util/lane_escape_util.h"

namespace apollo {
namespace planning {

bool LaneEscapeUtil::IsNeedEscape(ReferenceLineInfo& reference_line_info,
                  double min_distance_block_obs_to_junction,
                  bool enable_junction_borrow,
                  double passby_min_gap,
                  double passby_kappa_ratio,
                  int queue_check_count,
                  int stable_block_count) {
    AINFO << "IsNeedEscape";
    static std::pair<std::shared_ptr<Obstacle>, int> queue_sence_obs_info = std::make_pair(std::make_shared<Obstacle>(), 0);
    static std::pair<std::string, int> stable_block_obs_count = std::make_pair("", 0);
    if (std::fabs(reference_line_info.vehicle_state().linear_velocity())
        > common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param().max_abs_speed_when_stopped()) {
        AINFO << "Vehicle is not stopped.";
        return false;
    }

    if (reference_line_info.path_data().Empty()) {
        AINFO << "reference_line_info.path_data() is empty.";
        return false;
    }

    // if (reference_line_info.path_data().path_label().find("regular") != std::string::npos &&
    //     reference_line_info.path_data().blocking_obstacle_id().empty()) {
    //   AINFO << "find regular path: " << reference_line_info.path_data().path_label();
    //   return false;
    // }

    if (reference_line_info.SDistanceToDestination() < 20.0) {
        AINFO << "Distance to destination is less than 20m.";
        return false;
    }

    double min_distance = std::numeric_limits<double>::max();
    std::string stop_decision_obs_id;
    std::string block_obs_id;
    if (!GetClosestStopDecisionObs(reference_line_info, stop_decision_obs_id, min_distance)) {
        AINFO << "sweeper decide stop, don't need escape.";
        return false;
    }
    AINFO << "min_distance: " << min_distance;
    if (min_distance < FLAGS_min_stop_distance_obstacle + 2.0 && min_distance > 0) {
        block_obs_id = stop_decision_obs_id;
        AINFO << "block_obs_id: " << block_obs_id;
    } else {
        queue_sence_obs_info.first.reset();
        queue_sence_obs_info.second = 0;
        stable_block_obs_count.first.clear();
        stable_block_obs_count.second = 0;
        return false;
    }

    if (DistanceBlockingObstacleToJunction(reference_line_info, block_obs_id)
        < min_distance_block_obs_to_junction) {
        AINFO << "enable_junction_borrow is false and DistanceBlockingObstacleToIntersection is smaller than 22m.";
        return false;
    }
    if (!IsEnoughSpace(reference_line_info, block_obs_id, passby_min_gap, passby_kappa_ratio)) {
        AINFO << "IsEnoughSpace: false";
        return false;
    }

    bool is_queue = IsQueueSence(reference_line_info, block_obs_id, queue_check_count, queue_sence_obs_info);
    bool is_stable_block_obs = IsStableBlockObs(reference_line_info, block_obs_id, stable_block_count, stable_block_obs_count);

    if (is_queue || !is_stable_block_obs) {
        AINFO << "IsNeedEscape: false";
        return false;
    }

    queue_sence_obs_info.first.reset();
    queue_sence_obs_info.second = 0;
    stable_block_obs_count.first.clear();
    stable_block_obs_count.second = 0;

    return true;
}

bool LaneEscapeUtil::IsEnoughSpace(
        const ReferenceLineInfo& reference_line_info,
        const std::string& blocking_obstacle_id,
        const double &passby_min_gap,
        const double &passby_kappa_ratio) {
    bool enable_lane_borrow = false;
    bool lane_follow_enough_space = false;
    double check_s = reference_line_info.AdcSlBoundary().end_s();
    auto ref_point = reference_line_info.reference_line().GetNearestReferencePoint(check_s);
    const auto waypoint = ref_point.lane_waypoints().front();
    hdmap::LaneBoundaryType::Type lane_boundary_type = hdmap::LaneBoundaryType::UNKNOWN;
    auto ptr_lane_info = reference_line_info.LocateLaneInfo(check_s);
    if (ptr_lane_info == nullptr) {
        AWARN << "ptr_lane_info is null";
        return false;
    }
    if (!ptr_lane_info->lane().left_neighbor_forward_lane_id().empty()
        || !ptr_lane_info->lane().left_neighbor_reverse_lane_id().empty()) {
        AERROR << "check_s: " << check_s;
        lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
        if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW
            || lane_boundary_type == hdmap::LaneBoundaryType::DOUBLE_YELLOW
            || lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
            AINFO << "cannot borrow left lane";
        } else {
            enable_lane_borrow = true;
        }
    }
    AERROR << "check_s: " << check_s;
    if (!ptr_lane_info->lane().right_neighbor_forward_lane_id().empty()
        || !ptr_lane_info->lane().right_neighbor_reverse_lane_id().empty()) {
        lane_boundary_type = hdmap::RightBoundaryType(waypoint);
        if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW
            || lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
            AINFO << "cannot borrow right lane";
        } else {
            enable_lane_borrow = true;
        }
    }

    const Obstacle* block_obs = reference_line_info.path_decision().Find(blocking_obstacle_id);
    double ego_width = common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param().width();
    const auto obstacle_sl = block_obs->PerceptionSLBoundary();
    double obs_start_l = obstacle_sl.start_l();
    double obs_end_l = obstacle_sl.end_l();
    double check_start_s = obstacle_sl.start_s();
    double check_end_s = obstacle_sl.end_s();
    double obs_s = (check_start_s + check_end_s) / 2;
    double left_lane_width = 0.0;
    double right_lane_width = 0.0;
    reference_line_info.reference_line().GetLaneWidth(obs_s, &left_lane_width, &right_lane_width);
    const PathDecision& path_decision = reference_line_info.path_decision();
    // 0:start of obs l; 1:end of obs l
    std::vector<std::pair<double, int>> obs_l_range;
    for (const auto* ptr_obstacle_item : path_decision.obstacles().Items()) {
        const Obstacle* ptr_obstacle = path_decision.Find(ptr_obstacle_item->Id());
        if (ptr_obstacle == nullptr || !ptr_obstacle->IsStatic()
            || ptr_obstacle->IsVirtual()) {
            continue;
        }
        const SLBoundary& obs_sl_boundary = ptr_obstacle_item->PerceptionSLBoundary();
        if (obs_sl_boundary.start_s() > check_end_s || obs_sl_boundary.end_s() < check_start_s) {
            continue;
        }
        obs_l_range.emplace_back(std::make_pair(obs_sl_boundary.start_l(), 0));
        obs_l_range.emplace_back(std::make_pair(obs_sl_boundary.end_l(), 1));
        AINFO << "obstacle: " << ptr_obstacle->Id() << " " << obs_sl_boundary.start_l() << " "
              << obs_sl_boundary.end_l();
        AINFO << ptr_obstacle->DebugString();
    }
    std::sort(obs_l_range.begin(), obs_l_range.end(), [](std::pair<double, int>& p1, std::pair<double, int>& p2) {
        return p1.first < p2.first;
    });
    int gap_count = 0;
    std::vector<double> space_gap;
    for (size_t i = 0; i < obs_l_range.size(); ++i) {
        if (gap_count == 0) {
            space_gap.push_back(obs_l_range.at(i).first);
            AINFO << "space_gap: " << space_gap.back();
        }
        if (gap_count < 0) {
            AERROR << "check error";
            return false;
        }
        if (obs_l_range.at(i).second == 0) {
            gap_count++;
        } else if (obs_l_range.at(i).second == 1) {
            gap_count--;
        }
        if (gap_count == 0) {
            space_gap.push_back(obs_l_range.at(i).first);
            AINFO << "space_gap: " << space_gap.back();
        }
    }
    if (space_gap.empty()) {
        AINFO << "space_gap is empty";
        space_gap.push_back(left_lane_width);
    }
    double max_gap = right_lane_width + space_gap.front();
    AINFO << "max_gap: " << max_gap;
    for (int i = 2; i < space_gap.size(); i += 2) {
        double temp_gap = space_gap[i] - space_gap[i - 1];
        if (temp_gap > max_gap) {
            max_gap = temp_gap;
            AINFO << "max_gap: " << max_gap;
        }
    }
    max_gap = std::max(max_gap, left_lane_width - space_gap.back());
    double obs_reference_point_kappa = std::fabs(reference_line_info.reference_line().GetReferencePoint(obs_s).kappa());
    AINFO << "max_gap: " << max_gap << " obs_reference_point_kappa: " << obs_reference_point_kappa;
    if (max_gap - ego_width > passby_min_gap + obs_reference_point_kappa * passby_kappa_ratio) {
        lane_follow_enough_space = true;
    } else {
        AINFO << "lane_follow_enough_space: false, max_gap: " << max_gap << " ego_width: " << ego_width;
    }

    if (enable_lane_borrow || lane_follow_enough_space) {
        return true;
    } else {
        AINFO << "enable_lane_borrow: " << enable_lane_borrow
              << ", lane_follow_enough_space: " << lane_follow_enough_space;
        return false;
    }
}

bool LaneEscapeUtil::IsQueueSence(
        ReferenceLineInfo& reference_line_info,
        const std::string& blocking_obstacle_id,
        const int &queue_check_count,
        std::pair<std::shared_ptr<Obstacle>, int> &queue_obs_info) {
    if (blocking_obstacle_id.empty()) {
        ADEBUG << "There is no blocking obstacle.";
        return true;
    }
    const Obstacle* obs = reference_line_info.path_decision()->obstacles().Find(blocking_obstacle_id);
    if (obs == nullptr) {
        ADEBUG << "Blocking obstacle is no longer there.";
        return true;
    }

    if (obs->IsVirtual() || obs->PerceptionSLBoundary().start_s() < 0
        || obs->Perception().type() != perception::PerceptionObstacle::VEHICLE) {
        return false;
    }
    double obs_center_l = (obs->PerceptionSLBoundary().start_l() + obs->PerceptionSLBoundary().end_l()) / 2;
    if (obs_center_l < -1.5) {
        AINFO << "side parking car: " << obs->Id();
        return false;
    }

    if (!obs->IsStatic()) {
        AINFO << "moving obs: " << obs->Id();
        return true;
    }

    if (queue_obs_info.first == nullptr) {
        queue_obs_info.first = std::make_shared<Obstacle>(*obs);
        queue_obs_info.second = 1;
        return true;
    }

    if (queue_obs_info.first->Id() == obs->Id()) {
        if (queue_obs_info.second < queue_check_count) {
            queue_obs_info.second++;
            AINFO << "obs id: " << obs->Id() << ", queue sence is true.";
            return true;
        } else {
            // block obs static more than 10s
            AINFO << "obs id: " << obs->Id() << ", queue sence is false.";
            return false;
        }
    } else {
        queue_obs_info.first = std::make_shared<Obstacle>(*obs);
        queue_obs_info.second = 1;
        AINFO << "obs id: " << obs->Id() << ", queue sence is true.";
        return true;
    }
}

bool LaneEscapeUtil::IsStableBlockObs(ReferenceLineInfo& reference_line_info, const std::string& blocking_obstacle_id, const int &stable_block_count, std::pair<std::string, int> &stable_block_obs) {
    if (blocking_obstacle_id.empty()) {
        ADEBUG << "There is no blocking obstacle.";
        return false;
    }
    if (stable_block_obs.first.empty()) {
        stable_block_obs.first = blocking_obstacle_id;
        stable_block_obs.second = 1;
        return false;
    }

    if (stable_block_obs.first == blocking_obstacle_id) {
        if (stable_block_obs.second < stable_block_count) {
            stable_block_obs.second++;
            AINFO << "obs id: " << blocking_obstacle_id << ", stable is false.";
            return false;
        } else {
            // block obs static more than 3s
            AINFO << "obs id: " << blocking_obstacle_id << ", stable is true.";
            return true;
        }
    } else {
        stable_block_obs.first = blocking_obstacle_id;
        stable_block_obs.second = 1;
        AINFO << "obs id: " << blocking_obstacle_id << ", stable is false.";
        return false;
    }
}

bool LaneEscapeUtil::GetClosestStopDecisionObs(
        const ReferenceLineInfo& reference_line_info,
        std::string& stop_decision_obs_id,
        double& min_distance) {
    for (const auto& obs : reference_line_info.path_decision().obstacles().Items()) {

        if (obs->Id().find("SWEEPER_NUDGE_") != std::string::npos) {
            return false;
        }

        const Obstacle* ptr_obstacle = reference_line_info.path_decision().Find(obs->Id());
        ACHECK(ptr_obstacle != nullptr);
        double dist = ptr_obstacle->PerceptionSLBoundary().start_s() - reference_line_info.AdcSlBoundary().end_s();
        if (!ptr_obstacle->IsVirtual() &&
            ptr_obstacle->IsStatic() &&
            ptr_obstacle->LongitudinalDecision().has_stop() &&
            min_distance > dist) {
            min_distance = dist;
            stop_decision_obs_id = ptr_obstacle->Id();
        }
    }
    return true;
}

}  // namespace planning
}  // namespace apollo
