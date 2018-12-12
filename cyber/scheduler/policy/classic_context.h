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

#ifndef CYBER_SCHEDULER_POLICY_CLASSIC_CONTEXT_H_
#define CYBER_SCHEDULER_POLICY_CLASSIC_CONTEXT_H_

#include <array>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

#include "cyber/base/atomic_rw_lock.h"
#include "cyber/croutine/croutine.h"
#include "cyber/scheduler/processor_context.h"

namespace apollo {
namespace cyber {
namespace scheduler {

static constexpr uint32_t MAX_PRIO = 20;

class ClassicContext : public ProcessorContext {
 public:
  std::shared_ptr<CRoutine> NextRoutine() override;
  void Wait() override;
  void Shutdown() override;

  static void Notify();

  alignas(
      CACHELINE_SIZE) static std::array<base::AtomicRWLock, MAX_PRIO> rq_locks_;
  alignas(CACHELINE_SIZE) static std::array<
      std::vector<std::shared_ptr<croutine::CRoutine>>, MAX_PRIO> rq_;

 private:
  alignas(CACHELINE_SIZE) static std::mutex mtx_wq_;
  alignas(CACHELINE_SIZE) static std::condition_variable cv_wq_;

  std::chrono::steady_clock::time_point wake_time_;
  bool need_sleep_ = false;
};

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SCHEDULER_POLICY_CLASSIC_CONTEXT_H_
