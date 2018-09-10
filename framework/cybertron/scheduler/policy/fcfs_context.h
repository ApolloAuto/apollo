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

#ifndef CYBERTRON_SCHEDULER_POLICY_FCFS_CONTEXT_H_
#define CYBERTRON_SCHEDULER_POLICY_FCFS_CONTEXT_H_

#include <cstdint>
#include <future>
#include <limits>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "cybertron/base/atomic_rw_lock.h"
#include "cybertron/scheduler/policy/processor_context.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

class Processor;

class FCFSContext : public ProcessorContext {
 public:
  explicit FCFSContext(const std::shared_ptr<Processor>& processor)
      : ProcessorContext(processor) {}
  bool IsPriorInverse(uint64_t routine_id);

  std::shared_ptr<CRoutine> NextRoutine() override;
  bool Enqueue(const std::shared_ptr<CRoutine>& cr) override;
  bool RqEmpty() override;
 private:
  std::mutex mtx_run_queue_;
  double min_vfrequency_ = 0.0;
  double max_vfrequency_ = std::numeric_limits<double>::max();
  CRoutineList run_queue_;
};


}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_SCHEDULER_POLICY_FCFS_CONTEXT_H_
