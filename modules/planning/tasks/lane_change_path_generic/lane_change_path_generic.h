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

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include "modules/planning/tasks/lane_change_path_generic/proto/lane_change_path_generic.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
#include "modules/planning/park_data_center/park_data_center.h"

namespace apollo {
namespace planning {

enum SidePassDirection { LEFT_BORROW = 1, RIGHT_BORROW = 2 };
class LaneChangePathGeneric : public PathGeneration {
public:
    bool Init(
            const std::string& config_dir,
            const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

private:
    apollo::common::Status Process(Frame* frame, ReferenceLineInfo* reference_line_info) override;

    /**
     * @brief Calculate all path boundaries
     * @param boundary is calculated path boundaries
     */
    bool DecidePathBounds(std::vector<PathBoundary>* boundary);
    /**
     * @brief Optimize paths for each path boundary
     * @param path_boundaries is input path boundaries
     * @param candidate_path_data is output paths
     */
    bool OptimizePath(const std::vector<PathBoundary>& path_boundaries, std::vector<PathData>* candidate_path_data);
    /**
     * @brief Assess the feasibility of each path and select the best one
     * @param candidate_path_data is input paths
     * @param final_path is output the best path
     */
    bool AssessPath(std::vector<PathData>* candidate_path_data, PathData* final_path);
    /**
     * @brief Update lane change status
     */
    void UpdateLaneChangeStatus();
    /**
     * @brief Correct the boundary based on the starting point of lane change
     * @param path_bound is input path bound
     */
    void GetBoundaryFromLaneChangeForbiddenZone(PathBoundary* const path_bound);

    bool GetBoundaryFromNudgeDecision(PathBoundary* const path_bound);

    /**
     * @brief Calculate starting point of lane change
     * @param adc_frenet_s is adc current position
     * @param start_xy is output starting point of lane change
     */
    void GetLaneChangeStartPoint(
            const ReferenceLine& reference_line,
            double adc_frenet_s,
            common::math::Vec2d* start_xy);
    /**
     * @brief Determine if the space before and after changing lanes is safe
     */
    bool IsClearToChangeLane(ReferenceLineInfo* reference_line_info);
    /**
     * @brief Update Planning context lane change status
     */
    void UpdateStatus(double timestamp, ChangeLaneStatus::Status status_code, const std::string& path_id);
    /**
     * @brief Determine whether the obstacle meets the safe distance
     */
    bool HysteresisFilter(
            const double obstacle_distance,
            const double safe_distance,
            const double distance_buffer,
            const bool is_obstacle_blocking);
    void SetPathInfo(PathData* const path_data);

    bool CheckLastFrameSucceed(const apollo::planning::Frame* const last_frame);
    void GetSLPolygons(std::vector<SLPolygon>* polygons);

private:
    LaneChangePathGenericConfig config_;
    bool is_clear_to_change_lane_ = false;
    bool is_exist_lane_change_start_position_ = false;
    common::math::Vec2d lane_change_start_xy_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::LaneChangePathGeneric, Task)
}  // namespace planning
}  // namespace apollo
