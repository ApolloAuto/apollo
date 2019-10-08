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

#include "modules/storytelling/story_tellers/close_to_junction_teller.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/storytelling/frame_manager.h"

namespace apollo {
namespace storytelling {

using apollo::planning::ADCTrajectory;

void CloseToJunctionTeller::Init() {
  auto* manager = FrameManager::Instance();
  manager->CreateOrGetReader<ADCTrajectory>(FLAGS_planning_trajectory_topic);
}

void CloseToJunctionTeller::Update(Stories* stories) {
  // TODO(xiaoxq): Implement.
  // Log 'Enter CloseToJunction Story'
  // Log 'Exit CloseToJunction Story'
}

}  // namespace storytelling
}  // namespace apollo
