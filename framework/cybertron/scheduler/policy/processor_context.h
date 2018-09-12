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

#ifndef CYBERTRON_SCHEDULER_POLICY_PROCESSOR_CONTEXT_H_
#define CYBERTRON_SCHEDULER_POLICY_PROCESSOR_CONTEXT_H_

#include <future>
#include <limits>
#include <list>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "cybertron/base/atomic_hash_map.h"
#include "cybertron/base/atomic_rw_lock.h"
#include "cybertron/croutine/croutine.h"

namespace apollo {
namespace cybertron {
namespace scheduler {


using apollo::cybertron::base::AtomicHashMap;
using apollo::cybertron::base::AtomicRWLock;
using apollo::cybertron::base::ReadLockGuard;
using apollo::cybertron::base::WriteLockGuard;
using croutine::CRoutine;
using croutine::RoutineState;
using CRoutineList = std::list<std::shared_ptr<CRoutine>>;
using CRoutineMap = std::unordered_map<uint64_t, std::shared_ptr<CRoutine>>;

class Processor;
class ProcessorStat;

struct CommonState {
  bool running = false;
};

class ProcessorContext {
 public:
  explicit ProcessorContext(const std::shared_ptr<Processor>& processor)
      : processor_(processor), notified_(false) {}

  void ShutDown();
  void PrintStatistics();
  void PrintCRoutineStats();
  void UpdateProcessStat(ProcessorStat* stat);

  inline void SetId(const int& id) { proc_index_ = id; }
  inline bool GetState(const uint64_t& routine_id, RoutineState* state);
  inline bool SetState(const uint64_t& routine_id, const RoutineState& state);

  void NotifyProcessor(uint64_t croutine_id);
  void Push(const std::shared_ptr<CRoutine>& cr);
  bool Pop(uint64_t croutine_id, std::future<std::shared_ptr<CRoutine>>& fut);
  void RemoveCRoutine(uint64_t croutine_id);

  virtual bool RqEmpty() = 0;
  virtual std::shared_ptr<CRoutine> NextRoutine() = 0;

 protected:
  virtual bool IsPriorInverse(uint64_t routine_id) = 0;
  virtual bool Enqueue(const std::shared_ptr<CRoutine>& cr) = 0;

//  std::mutex mtx_cr_map_;
  AtomicRWLock rw_lock_;
  CRoutineMap cr_map_;
  AtomicHashMap<uint64_t, std::promise<std::shared_ptr<CRoutine>>> pop_list_;
  std::shared_ptr<Processor> processor_;

  bool stop_ = false;
  std::atomic<bool> notified_;
  uint32_t index_ = 0;
  uint32_t status_;
  int proc_index_ = -1;

};

bool ProcessorContext::GetState(const uint64_t& routine_id,
                                RoutineState* state) {
  ReadLockGuard lg(rw_lock_);
  auto it = cr_map_.find(routine_id);
  if (it != cr_map_.end()) {
    *state = it->second->State();
    return true;
  }
  return false;
}

bool ProcessorContext::SetState(const uint64_t& routine_id,
                                const RoutineState& state) {
  ReadLockGuard lg(rw_lock_);
  auto it = cr_map_.find(routine_id);
  if (it != cr_map_.end()) {
    it->second->SetState(state);
    return true;
  }
  return false;
}

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_SCHEDULER_POLICY_PROCESSOR_CONTEXT_H_
