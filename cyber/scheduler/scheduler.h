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

#ifndef CYBER_SCHEDULER_SCHEDULER_H_
#define CYBER_SCHEDULER_SCHEDULER_H_

#include <unistd.h>
#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "cyber/base/atomic_rw_lock.h"
#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/common/types.h"
#include "cyber/croutine/croutine.h"
#include "cyber/croutine/routine_factory.h"
#include "cyber/proto/choreography_conf.pb.h"
#include "cyber/proto/scheduler_conf.pb.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::croutine::CRoutine;
using apollo::cyber::croutine::RoutineFactory;
using apollo::cyber::data::DataVisitorBase;

class ProcessorContext;

class Scheduler {
 public:
  Scheduler() : stop_(false) {}
  virtual ~Scheduler() {}
  static Scheduler* Instance();
  bool CreateTask(const RoutineFactory& factory, const std::string& name);
  bool CreateTask(std::function<void()>&& func, const std::string& name,
                  std::shared_ptr<DataVisitorBase> visitor = nullptr);
  virtual bool RemoveTask(const std::string& name) = 0;
  bool NotifyTask(uint64_t crid);
  void ShutDown();
  uint32_t TaskPoolSize() { return task_pool_size_; }

 protected:
  virtual bool DispatchTask(const std::shared_ptr<CRoutine>) = 0;
  virtual bool NotifyProcessor(uint64_t crid) = 0;
  void ParseCpuset(const std::string&, std::vector<int>*);

  std::vector<std::shared_ptr<ProcessorContext>> pctxs_;
  uint32_t proc_num_ = 0;
  uint32_t task_pool_size_ = 0;
  uint32_t cpu_binding_start_index_ = 0;
  std::atomic<bool> stop_;
};

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SCHEDULER_SCHEDULER_H_
