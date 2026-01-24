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
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common_msgs/external_command_msgs/precise_parking_command.pb.h"
#include "modules/planning/planning_interface_base/scenario_base/stage.h"
#include "modules/planning/planning_open_space/utils/open_space_roi_util.h"
#include "modules/planning/scenarios/precise_parking/precise_parking_scenario.h"

namespace apollo {
namespace planning {

class StagePreciseParking : public Stage {
public:
    bool Init(
            const StagePipeline &config,
            const std::shared_ptr<DependencyInjector> &injector,
            const std::string &config_dir,
            void *context);

    StageResult Process(const common::TrajectoryPoint &planning_init_point, Frame *frame) override;

private:
    bool GetOpenspaceRoi(Frame *frame);
    void SetOrigin(const std::vector<Vec2d> &points, const Vec2d &ego_pos, Frame *frame);
    void GetEndPose(Frame *frame);
    bool FuseLineSegments(std::vector<std::vector<common::math::Vec2d>> &line_segments_vec);
    bool FormulateBoundaryConstraints(
            const std::vector<std::vector<common::math::Vec2d>> &roi_parking_boundary,
            Frame *const frame);
    StageResult FinishStage(Frame *frame);
    bool LoadObstacleInVertices(
            const std::vector<std::vector<common::math::Vec2d>> &roi_parking_boundary,
            Frame *const frame);

    PreciseParkingContext *precise_parking_context_;
    common::math::Vec2d origin_point_;
    double origin_heading_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::StagePreciseParking, Stage)

}  // namespace planning
}  // namespace apollo
