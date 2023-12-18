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

#include "modules/planning/planning_interface_base/task_base/task.h"

namespace apollo {
namespace planning {

class TrajectoryFallbackTask : public Task {
 public:
  common::Status Execute(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;

 private:
  virtual void GenerateFallbackPath(Frame* frame,
                                    ReferenceLineInfo* reference_line_info);

  virtual SpeedData GenerateFallbackSpeed(const EgoInfo* ego_info,
                                          const double stop_distance = 0.0) = 0;

  void GenerateFallbackPathProfile(const ReferenceLineInfo* reference_line_info,
                                   PathData* path_data);

  bool RetrieveLastFramePathProfile(
      const ReferenceLineInfo* reference_line_info, const Frame* frame,
      PathData* path_data);
  /**
   * @brief Modify the deceleration before stopping so that control module can
   * follow up.
   *
   * @param speed_data_ptr The speed data to be modified.
   */
  void AmendSpeedDataForControl(SpeedData* speed_data_ptr);
};

}  // namespace planning
}  // namespace apollo
