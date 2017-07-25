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
 * @file master_state_machine.h
 * @description:
 *    header file of master state machine.
 **/

#ifndef MODULES_PLANNING_STATE_MACHINE_MASTER_STATE_MACHINE_H_
#define MODULES_PLANNING_STATE_MACHINE_MASTER_STATE_MACHINE_H_

namespace apollo {
namespace planning {

class MasterStateMachine {
 public:
  enum MasterState {
      INIT = 0,
      CRUISE = 1,
      CHANGING_LANE = 2,
      ESTOP = 3,
      WAIT = 4,
      FINISH = 5,
      OVERTAKE = 6,
    };

  MasterStateMachine() = default;
  void set_state(MasterState state);
  MasterState state() const;

 private:
  MasterState state_ = MasterState::INIT;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_STATE_MACHINE_MASTER_STATE_MACHINE_H_
