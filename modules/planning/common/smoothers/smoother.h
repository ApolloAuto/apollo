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
 * @file
 **/

#pragma once

#include "modules/common/status/status.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/planning/common/frame.h"
#include "modules/common_msgs/planning_msgs/decision.pb.h"

namespace apollo {
namespace planning {

class Smoother {
 public:
  Smoother() = default;
  virtual ~Smoother() = default;

  apollo::common::Status Smooth(const FrameHistory* frame_history,
                                const Frame* current_frame,
                                ADCTrajectory* const current_trajectory_pb);

 private:
  bool IsCloseStop(const common::VehicleState& vehicle_state,
                   const MainStop& main_stop);
};

}  // namespace planning
}  // namespace apollo
