/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
 * @file perception_proxy.cc
 **/

#include "modules/planning/proxy/perception_proxy.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

void PerceptionProxy::add_perception_obstacles(
    const apollo::perception::PerceptionObstacles& perception_obstacles) {
  if (_perception_frame.size() >
      static_cast<std::size_t>(FLAGS_max_frame_size)) {
    _perception_frame.pop_front();
  }
  _perception_frame.emplace_back(std::move(perception_obstacles));
  return;
}

apollo::perception::PerceptionObstacles*
PerceptionProxy::get_latest_perception_frame() {
  if (_perception_frame.size() == 0) {
    return nullptr;
  }
  return &(*(_perception_frame.rbegin()));
}

const std::list<apollo::perception::PerceptionObstacles>&
PerceptionProxy::perception_frame() const {
  return _perception_frame;
}

}  // namespace planning
}  // namespace apollo
