/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @file stage_creep.h
 **/

#pragma once

#include <memory>
#include <string>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/scenario_base/base_stage_creep.h"

namespace apollo {
namespace planning {

class CreepDecider;

class StopSignUnprotectedStageCreep : public BaseStageCreep {
 public:
  bool Init(const StagePipeline& config,
            const std::shared_ptr<DependencyInjector>& injector,
            const std::string& config_dir, void* context) override;

  StageResult Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;

 private:
  /**
   * @brief Get the config of creep stage from ScenarioContext, to be overwrited
   * by the sub classes.
   *
   * @return config of creep stage
   */
  const CreepStageConfig& GetCreepStageConfig() const override;

  /**
   * @brief Get the overlap id of stage and the stop line distance according to
   * the frame and reference line information.
   *
   * @param frame current frame information
   * @param reference_line_info current reference line information
   * @param overlap_end_s end distance mapped to reference line of the overlap
   * @param overlap_id overlap id of current stage
   * @return return true if find a valid overlap
   */
  bool GetOverlapStopInfo(Frame* frame, ReferenceLineInfo* reference_line_info,
                          double* overlap_end_s,
                          std::string* overlap_id) const override;

  StageResult FinishStage();
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::planning::StopSignUnprotectedStageCreep, Stage)

}  // namespace planning
}  // namespace apollo
