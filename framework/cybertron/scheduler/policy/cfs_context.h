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

#ifndef CYBERTRON_SCHEDULER_POLICY_CFS_CONTEXT_H_
#define CYBERTRON_SCHEDULER_POLICY_CFS_CONTEXT_H_

#include <cstdint>
#include <future>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "cybertron/scheduler/policy/processor_context.h"

namespace apollo {
namespace cybertron {
namespace croutine {
class CRoutine;
}

namespace scheduler {

class Processor;

using croutine::CRoutine;

class CFSContext : public ProcessorContext {
 public:
  explicit CFSContext(const std::shared_ptr<Processor>& processor)
      : ProcessorContext(processor) {}
  std::shared_ptr<CRoutine> NextRoutine();
  bool NextRoutine(const std::shared_ptr<CRoutine>& cr);
  bool Enqueue(const std::shared_ptr<CRoutine>& cr);
  bool IsPriorInverse(uint64_t routine_id);
  bool RqEmpty();

 private:
  std::mutex mtx_run_queue_;
  CRoutineList wait_queue_;
  CRoutineList run_queue_;
  std::multimap<double, std::shared_ptr<CRoutine>> rb_map_;
  double max_vruntime_ = std::numeric_limits<double>::max();
  double min_vruntime_ = 0;
};

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_SCHEDULER_POLICY_CFS_CONTEXT_H_
