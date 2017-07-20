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
 * @file:
 **/

#ifndef MODULES_PLANNING_COMMON_DATA_CENTER_H_
#define MODULES_PLANNING_COMMON_DATA_CENTER_H_

#include <list>
#include <memory>
#include <unordered_map>

#include "modules/common/macro.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/environment.h"
#include "modules/planning/state_machine/master_state_machine.h"

namespace apollo {
namespace planning {

class DataCenter {
 public:
  ~DataCenter() = default;
  Frame* frame(const uint32_t sequence_num) const;
 public:
  apollo::common::Status init_frame(const uint32_t sequence_num);
  Frame* current_frame() const;
  void save_frame();

  Environment* mutable_environment();
  MasterStateMachine* mutable_master() const;

  const Frame* last_frame() const;
 private:
  std::unordered_map<uint32_t, std::unique_ptr<Frame>> _frames;
  std::list<uint32_t> _sequence_queue;
  Environment _environment;
  std::unique_ptr<Frame> _frame = nullptr;
  std::unique_ptr<MasterStateMachine> _master = nullptr;
 private:
  DECLARE_SINGLETON(DataCenter);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_DATA_CENTER_H_
