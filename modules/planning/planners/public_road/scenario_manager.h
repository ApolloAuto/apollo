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

#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/planning/planners/public_road/proto/planner_config.pb.h"

namespace apollo {
namespace planning {

class DependencyInjector;
class Frame;
}  // namespace planning
}  // namespace apollo

namespace apollo {
namespace planning {

class Scenario;

class ScenarioManager final {
 public:
  bool Init(const std::shared_ptr<DependencyInjector>& injector,
            const PlannerPublicRoadConfig& planner_config);

  Scenario* mutable_scenario() { return current_scenario_.get(); }

  DependencyInjector* injector() { return injector_.get(); }

  void Update(const common::TrajectoryPoint& ego_point, Frame* frame);

  void Reset(Frame* frame);

 private:
  std::shared_ptr<DependencyInjector> injector_;
  std::shared_ptr<Scenario> current_scenario_;
  std::shared_ptr<Scenario> default_scenario_type_;
  std::vector<std::shared_ptr<Scenario>> scenario_list_;
  bool init_ = false;
};

}  // namespace planning
}  // namespace apollo
