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
 * @file nudge_info.h
 **/

#pragma once

#include <limits>
#include <list>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/math/polygon2d.h"
#include "modules/planning/planning_base/common/obstacle.h"
#include "modules/planning/planning_base/common/sl_polygon.h"

namespace apollo {
namespace planning {

using common::SLPoint;
using common::math::Polygon2d;

struct RemainNudgeSpace {
    double left_lane_remain = std::numeric_limits<double>::min();
    double left_road_remain = std::numeric_limits<double>::min();
    double right_lane_remain = std::numeric_limits<double>::min();
    double right_road_remain = std::numeric_limits<double>::min();
    hdmap::LaneBoundaryType::Type left_lane_boundary_type = hdmap::LaneBoundaryType::CURB;
    hdmap::LaneBoundaryType::Type right_lane_boundary_type = hdmap::LaneBoundaryType::CURB;
};

enum class NudgeType {
    NONE = 0,
    LEFT = 1,   // left cross line
    RIGHT = 2,  // right cross line
    WAITING = 3,
};

struct NudgeObstacleInfo {
    Polygon2d origin_polygon;
    Polygon2d expand_polygon;
    SLBoundary sl_boundary;
    RemainNudgeSpace remain_nudge_space;
    bool is_passable = false;
    bool is_all_on_lane = false;
    double nudge_probability = 0.0;
    double tracking_time = 0.0;
    double start_track_time = std::numeric_limits<double>::max();
    double last_track_time = 0;
    NudgeType nudge_type = NudgeType::NONE;

    double obs_tracking_time() {
        return last_track_time - start_track_time;
    }

    void Clear() {
        nudge_probability = 0.0;
        tracking_time = 0.0;
        nudge_type = NudgeType::NONE;
    }

    void PrintPolygonInfo(std::string obs_id) {
        if (origin_polygon.points().size() > 0) {
            PrintCurves print_curve;
            for (const auto& p : origin_polygon.points()) {
                print_curve.AddPoint(obs_id + "_BlockObsPolygon", p.x(), p.y());
            }
            print_curve.AddPoint(
                    obs_id + "_BlockObsPolygon", origin_polygon.points()[0].x(), origin_polygon.points()[0].y());
            print_curve.PrintToLog();
        }
    }
};

class NudgeInfo {
public:
    NudgeInfo() = default;

    void set_enable(bool is_enable);
    bool is_enable() const;

    void set_is_merge_block_obs(bool is_merge);
    bool is_merge_block_obs() const;

    const std::unordered_map<std::string, NudgeObstacleInfo>& tracking_nudge_obs_info() const;
    std::unordered_map<std::string, NudgeObstacleInfo>* mutable_tracking_nudge_obs_info();

    const std::vector<SLPolygon>& block_sl_polygons() const;
    std::vector<SLPolygon>* mutable_block_sl_polygons();

    const std::set<std::string>& update_ids() const;
    std::set<std::string>* mutable_update_ids();

    const std::vector<std::vector<SLPoint>>& extra_nudge_key_points() const;
    std::vector<std::vector<SLPoint>>* mutable_extra_nudge_key_points();

    bool IsObsIgnoreNudgeDecision(std::string obs_id, double obs_start_s, double check_dis) const;
    bool NeedCheckObsCollision(std::string obs_id) const;

    void GetFirstBlockSLPolygon(std::string obs_id, SLPolygon* obs_sl_polygon) const;

    void SortBlockSLPolygons();

    void PrintDebugInfo();
    bool GetInterpolatedNudgeL(bool is_left_nudge, double s, double* nudge_l) const;

private:
    bool is_enable_ = false;
    NudgeType nudge_type_ = NudgeType::NONE;
    bool is_merge_block_obs_ = false;
    std::unordered_map<std::string, NudgeObstacleInfo> tracking_nudge_obs_info_;
    std::set<std::string> update_ids_;

    std::vector<SLPolygon> block_sl_polygons_;

    std::vector<std::vector<SLPoint>> extra_nudge_key_points_;
};

}  // namespace planning
}  // namespace apollo
