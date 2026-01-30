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

/**
 * @file
 **/

#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "modules/planning/tasks/obstacle_nudge_decider/proto/obstacle_nudge_decider.pb.h"

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/park_data_center/park_data_center.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/common/util/util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/task.h"

namespace apollo {
namespace planning {

struct VehicleConfig {
    double width = 0.0;
    double length = 0.0;
    double front_edge_to_center = 0.0;
};

class NudgeCalculation {
public:
    explicit NudgeCalculation(const ObstacleNudgeDeciderConfig& cross_line_nudge_config);
    void BuildNudgeDecisionWithObs(
            Frame* const current_frame,
            const Frame* last_frame,
            ReferenceLineInfo* const reference_line_info);
    void set_in_nudge_state(bool state);

    void BuildStaticTrackingObs(
            ReferenceLineInfo* const reference_line_info,
            const Obstacle& obstacle,
            NudgeInfo* const nudge_info);
    bool CalcObsRemainSpace(
            ReferenceLineInfo* const reference_line_info,
            const Obstacle& obstacle,
            RemainNudgeSpace* remain_space);

    bool MatchTrackingObsInfo(
            const Obstacle& obstacle,
            NudgeInfo* const nudge_info,
            bool is_passable,
            RemainNudgeSpace remain_nudge_space,
            bool is_all_on_lane);

    bool MergeObsPolygon(ReferenceLineInfo* const reference_line_info, NudgeInfo* const nudge_info);
    void EraseObsInNudgeinfo(NudgeInfo* const nudge_info, std::string id);

    bool IsProbEnoughToNudge(NudgeInfo* const nudge_info, bool is_merge_obs = false);
    void CalcNudgeExtraSpace(ReferenceLineInfo* const reference_line_info, NudgeInfo* const nudge_info);

private:
    bool IsWithinNudgeScopeObstacle(ReferenceLineInfo* const reference_line_info, const Obstacle& obstacle);
    void UpdateTrackingObsInfo(
            NudgeObstacleInfo* const track_obs_info,
            const Obstacle& obstacle,
            bool is_passable,
            RemainNudgeSpace remain_nudge_space,
            bool is_all_on_lane);
    void GetRefLineLaneWidth(
            const ReferenceLine& reference_line,
            const double adc_s,
            const double search_s,
            double* const lane_left_width,
            double* const lane_right_width);

private:
    ObstacleNudgeDeciderConfig config_;
    VehicleConfig veh_config_;
    SLBoundary adc_boundary_;
    bool is_in_nudge_state_ = false;
    double check_forward_dis_ = 0;
    double dis2end_ = std::numeric_limits<double>::max();
};

}  // namespace planning
}  // namespace apollo
