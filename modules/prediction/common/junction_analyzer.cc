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

#include <queue>

#include "modules/prediction/common/junction_analyzer.h"

namespace apollo {
namespace prediction {

JunctionAnalyzer::JunctionAnalyzer(const std::string& junction_id) {
  // TODO(all) implement
}

std::vector<JunctionExit> JunctionAnalyzer::GetJunctionExits(
    const std::string& start_lane_id) {
  // TODO(all) implement
  std::vector<JunctionExit> junction_exits;
  return junction_exits;
}

bool JunctionAnalyzer::IsExitLane(const std::string& lane_id) {
  // TODO(kechxu) implement
  return false;
}

}  // namespace prediction
}  // namespace apollo
