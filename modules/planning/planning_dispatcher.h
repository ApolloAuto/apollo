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

#ifndef MODULES_PLANNING_PLANNING_DISPATCHER_H_
#define MODULES_PLANNING_PLANNING_DISPATCHER_H_

#include <memory>
#include <string>

#include "modules/common/apollo_app.h"
#include "modules/common/status/status.h"
#include "modules/common/util/thread_pool.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/navi_planning.h"
#include "modules/planning/planning_base.h"
#include "modules/planning/std_planning.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class planning
 *
 * @brief PlanningDispatcher module main class.
 */
class PlanningDispatcher final : public common::ApolloApp {
 public:
  PlanningDispatcher() = default;
  virtual ~PlanningDispatcher();

  common::Status Init() override {
    common::util::ThreadPool::Init(FLAGS_max_planning_thread_pool_size);
    Dispatch();
    return planning_base_->Init();
  }

  common::Status Start() override { return planning_base_->Start(); }

  void Stop() override { planning_base_->Stop(); }

  virtual void RunOnce() { planning_base_->RunOnce(); }

  std::string Name() const override { return planning_base_->Name(); }

 private:
  void Dispatch() {
    if (FLAGS_use_navigation_mode) {
      planning_base_ = std::unique_ptr<PlanningBase>(new NaviPlanning());
    } else {
      planning_base_ = std::unique_ptr<PlanningBase>(new StdPlanning());
    }
  }

  std::unique_ptr<PlanningBase> planning_base_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_PLANNING_DISPATCHER_H_
