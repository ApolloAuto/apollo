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

#ifndef MODULES_PLANNING_PLANNING_INTERFACE_H_
#define MODULES_PLANNING_PLANNING_INTERFACE_H_

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/apollo_app.h"
#include "modules/planning/proto/planning.pb.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class PlanningInterface
 *
 * @brief Interface of the planning module
 */
class PlanningInterface : public apollo::common::ApolloApp {
 public:
  /**
   * @brief main logic of the planning module, runs periodically triggered by
   * timer.
   */
  virtual void RunOnce() = 0;

  /**
   * @brief Fill the header and publish the planning message.
   */
  void Publish(const double timestamp, planning::ADCTrajectory* trajectory) {
    using apollo::common::adapter::AdapterManager;
    AdapterManager::FillPlanningHeader(Name(), timestamp, trajectory);
    AdapterManager::PublishPlanning(*trajectory);
  }
};

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_PLANNING_INTERFACE_H_ */
