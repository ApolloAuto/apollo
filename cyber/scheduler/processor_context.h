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

#ifndef CYBER_SCHEDULER_POLICY_PROCESSOR_CONTEXT_H_
#define CYBER_SCHEDULER_POLICY_PROCESSOR_CONTEXT_H_

#include <future>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "cyber/base/atomic_hash_map.h"
#include "cyber/base/atomic_rw_lock.h"
#include "cyber/base/macros.h"
#include "cyber/croutine/croutine.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::base::AtomicHashMap;
using apollo::cyber::base::AtomicRWLock;
using apollo::cyber::base::ReadLockGuard;
using apollo::cyber::base::WriteLockGuard;
using croutine::CRoutine;
using croutine::RoutineState;
using CRoutineContainer =
    std::unordered_map<uint64_t, std::shared_ptr<CRoutine>>;

class Processor;

class ProcessorContext {
 public:
  ProcessorContext() {}
  virtual ~ProcessorContext() {}

  void ShutDown();

  inline bool get_state(const uint64_t& cr_id, RoutineState* state);
  inline bool set_state(const uint64_t& cr_id, const RoutineState& state);

  void BindProc(const std::shared_ptr<Processor>& processor) {
    processor_ = processor;
  }
  std::shared_ptr<Processor> Proc() { return processor_; }

  virtual void Notify(uint64_t cr_id);
  virtual bool DispatchTask(const std::shared_ptr<CRoutine>) = 0;
  virtual bool Enqueue(const std::shared_ptr<CRoutine>) = 0;
  void RemoveCRoutine(uint64_t cr_id);
  int RqSize();

  virtual std::shared_ptr<CRoutine> NextRoutine() = 0;

 protected:
  alignas(CACHELINE_SIZE) CRoutineContainer cr_container_;
  alignas(CACHELINE_SIZE) AtomicRWLock rw_lock_;
  alignas(CACHELINE_SIZE) std::shared_ptr<Processor> processor_ = nullptr;
  alignas(CACHELINE_SIZE) std::atomic_flag notified_ = ATOMIC_FLAG_INIT;

  bool stop_ = false;
  uint32_t index_ = 0;
  uint32_t status_;
};

bool ProcessorContext::get_state(const uint64_t& cr_id, RoutineState* state) {
  ReadLockGuard<AtomicRWLock> lg(rw_lock_);
  auto it = cr_container_.find(cr_id);
  if (it != cr_container_.end()) {
    *state = it->second->state();
    return true;
  }
  return false;
}

bool ProcessorContext::set_state(const uint64_t& cr_id,
                                 const RoutineState& state) {
  ReadLockGuard<AtomicRWLock> lg(rw_lock_);
  auto it = cr_container_.find(cr_id);
  if (it != cr_container_.end()) {
    it->second->set_state(state);
    return true;
  }
  return false;
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SCHEDULER_POLICY_PROCESSOR_CONTEXT_H_
