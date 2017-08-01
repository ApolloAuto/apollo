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
 * @file planning_state.h
 **/

#ifndef MODULES_PLANNING_COMMON_PLANNING_STATE_H_
#define MODULES_PLANNING_COMMON_PLANNING_STATE_H_

#include <mutex>

#include "modules/common/macro.h"

namespace apollo {
namespace planning {

class PlanningState {
 public:
  enum StateCode {
    UNKNOWN = 0,
    INIT = 1,
    CRUISE = 2,
    CHANGING_LANE = 3,
    ESTOP = 4,
    FINISH = 6,
  };

  void SetState(StateCode state);
  StateCode State();

 private:
  StateCode state_ = StateCode::UNKNOWN;

  std::mutex mutex_;

  DECLARE_SINGLETON(PlanningState);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_PLANNING_STATE_H_
