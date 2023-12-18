/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/scenario_base/stage.h"

namespace apollo {
namespace planning {
class Frame;
}
}  // namespace apollo

namespace apollo {
namespace planning {

class ParkAndGoStageCheck : public Stage {
 public:
  StageResult Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;

 private:
  StageResult FinishStage(const bool success);

  bool CheckObstacle(const ReferenceLineInfo& reference_line_info);

  void ADCInitStatus();
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::ParkAndGoStageCheck,
                                     Stage)

}  // namespace planning
}  // namespace apollo
