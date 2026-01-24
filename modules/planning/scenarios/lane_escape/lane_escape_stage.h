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

/**
 * @file
 **/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/status/status.h"
#include "modules/planning/planning_interface_base/scenario_base/stage.h"
#include "modules/planning/scenarios/lane_escape/lane_escape_scenario.h"
#include "modules/planning/scenarios/lane_escape/proto/lane_escape_scenario.pb.h"

namespace apollo {
namespace planning {

class LaneEscapeParkStage : public Stage {
public:
    bool Init(
            const StagePipeline& config,
            const std::shared_ptr<DependencyInjector>& injector,
            const std::string& config_dir,
            void* context);
    StageResult Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) override;
    bool HasCollision(Frame* frame, const common::math::Box2d& adc_box);
    bool GenerateStraightReversePath(PathData* path_data, const common::TrajectoryPoint& init_point);

private:
    StageResult FinishStage();
    bool need_to_stop_ = false;
    int success_path_times_ = 0;
    int stop_check_count_ = 0;
    std::shared_ptr<common::PathPoint> stop_point_ = nullptr;
    ScenarioLaneEscapeConfig scenario_config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::LaneEscapeParkStage, Stage)

}  // namespace planning
}  // namespace apollo
