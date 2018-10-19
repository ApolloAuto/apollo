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

#include <functional>
#include <map>
#include <memory>
#include <unordered_map>

#include "cyber/scheduler/processor_context.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using croutine::CRoutine;

class ClassicContext : public ProcessorContext {
 public:
  std::shared_ptr<CRoutine> NextRoutine();
  bool Enqueue(const std::shared_ptr<CRoutine>& cr) override;
  bool RqEmpty() override;
  void Notify(uint64_t tid) override;
  bool EnqueueAffinityRoutine(const std::shared_ptr<CRoutine>& cr) {
    return false;
  }

 private:
  static std::mutex mtx_taskq_;
  static std::mutex mtx_rq_;
  static std::unordered_multimap<uint64_t, std::shared_ptr<CRoutine>> taskq_;
  static std::multimap<uint32_t, std::shared_ptr<CRoutine>,
                       std::greater<uint32_t>>
      rq_;
};

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SCHEDULER_POLICY_CLASSIC_CONTEXT_H_
