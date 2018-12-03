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

#ifndef CYBER_SCHEDULER_PROCESSOR_H_
#define CYBER_SCHEDULER_PROCESSOR_H_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "cyber/croutine/croutine.h"
#include "cyber/croutine/routine_context.h"
#include "cyber/proto/scheduler_conf.pb.h"

namespace apollo {
namespace cyber {
namespace scheduler {

class ProcessorContext;

using croutine::CRoutine;
using croutine::RoutineContext;

class Processor {
 public:
  Processor();
  ~Processor();

  void Run();
  void SetAffinity(const std::vector<int>&, const std::string&, int);
  void SetSchedPolicy(std::string spolicy, int sched_priority);
  void Stop() {
    if (!running_.exchange(false)) {
      return;
    }
  }

  void BindContext(const std::shared_ptr<ProcessorContext>& context) {
    context_ = context;
  }

 private:
  std::shared_ptr<ProcessorContext> context_;
  std::shared_ptr<RoutineContext> routine_context_ = nullptr;

  std::condition_variable cv_ctx_;
  std::mutex mtx_ctx_;
  std::thread thread_;

  std::atomic<bool> running_{false};
  std::atomic<pid_t> tid_{-1};
};

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SCHEDULER_PROCESSOR_H_
