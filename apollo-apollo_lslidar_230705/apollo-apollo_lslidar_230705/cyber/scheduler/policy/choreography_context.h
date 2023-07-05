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

#ifndef CYBER_SCHEDULER_POLICY_CHOREOGRAPHY_CONTEXT_H_
#define CYBER_SCHEDULER_POLICY_CHOREOGRAPHY_CONTEXT_H_

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "cyber/base/atomic_rw_lock.h"
#include "cyber/croutine/croutine.h"
#include "cyber/scheduler/processor_context.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::base::AtomicRWLock;
using croutine::CRoutine;

class ChoreographyContext : public ProcessorContext {
 public:
  bool RemoveCRoutine(uint64_t crid);
  std::shared_ptr<CRoutine> NextRoutine() override;

  bool Enqueue(const std::shared_ptr<CRoutine>&);
  void Notify();
  void Wait() override;
  void Shutdown() override;

 private:
  std::mutex mtx_wq_;
  std::condition_variable cv_wq_;
  int notify = 0;

  AtomicRWLock rq_lk_;
  std::multimap<uint32_t, std::shared_ptr<CRoutine>, std::greater<uint32_t>>
      cr_queue_;
};

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SCHEDULER_POLICY_CHOREOGRAPHY_CONTEXT_H_
