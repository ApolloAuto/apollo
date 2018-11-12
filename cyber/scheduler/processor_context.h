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

using apollo::cyber::base::AtomicRWLock;
using croutine::CRoutine;
using croutine::RoutineState;

class Processor;

class ProcessorContext {
 public:
  ProcessorContext() {}
  virtual ~ProcessorContext() {}
  void ShutDown();
  void BindProc(const std::shared_ptr<Processor>& processor) {
    processor_ = processor;
  }
  virtual std::shared_ptr<CRoutine> NextRoutine() = 0;

 protected:
  alignas(CACHELINE_SIZE) std::shared_ptr<Processor> processor_ = nullptr;
  alignas(CACHELINE_SIZE) std::atomic_flag notified_ = ATOMIC_FLAG_INIT;
  bool stop_ = false;
  uint32_t index_ = 0;
  uint32_t status_;
};

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SCHEDULER_POLICY_PROCESSOR_CONTEXT_H_
