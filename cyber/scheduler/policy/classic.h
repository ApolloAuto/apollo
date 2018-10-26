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

#ifndef CYBER_SCHEDULER_POLICY_CLASSIC_H_
#define CYBER_SCHEDULER_POLICY_CLASSIC_H_

#include <array>
#include <functional>
#include <memory>
#include <vector>

#include "cyber/base/atomic_rw_lock.h"
#include "cyber/scheduler/processor_context.h"

#define MAX_SCHED_PRIORITY 20

namespace apollo {
namespace cyber {
namespace scheduler {

using croutine::CRoutine;
using apollo::cyber::base::AtomicRWLock;
using apollo::cyber::base::ReadLockGuard;
using apollo::cyber::base::WriteLockGuard;

class ClassicContext : public ProcessorContext {
 public:
  bool DispatchTask(const std::shared_ptr<CRoutine>) override;
  bool Enqueue(const std::shared_ptr<CRoutine> cr) override;
  bool RqEmpty() override;
  std::shared_ptr<CRoutine> NextRoutine() override;


 private:
  static std::array<AtomicRWLock, MAX_SCHED_PRIORITY> rw_locks_;
  static std::array<std::vector<std::shared_ptr<CRoutine>>,
      MAX_SCHED_PRIORITY> rq_;
};

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SCHEDULER_POLICY_CLASSIC_H_
