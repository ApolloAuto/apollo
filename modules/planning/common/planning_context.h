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
 * @file
 */

#ifndef MODULES_PLANNING_COMMON_PLANNING_CONTEXT_H_
#define MODULES_PLANNING_COMMON_PLANNING_CONTEXT_H_

#include <string>

#include "modules/common/macro.h"
#include "modules/common/proto/drive_state.pb.h"
#include "modules/common/proto/pnc_point.pb.h"

#include "modules/planning/proto/planning_status.pb.h"

/**
 * @brief PlanningContext is the runtime context in planning. It is
 * persistant across multiple frames.
 */
namespace apollo {
namespace planning {

class PlanningContext {
 public:
  void Clear();

  PlanningStatus* GetPlanningStatus() { return &planning_status_; }

 private:
  PlanningStatus planning_status_;

  // this is a sigleton class
  DECLARE_SINGLETON(PlanningContext);
};

void DumpPlanningContext();

inline PlanningStatus* GetPlanningStatus() {
  return PlanningContext::instance()->GetPlanningStatus();
}

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_PLANNING_CONTEXT_H_
