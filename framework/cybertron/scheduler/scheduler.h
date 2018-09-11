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

#ifndef CYBERTRON_SCHEDULER_SCHEDULER_H_
#define CYBERTRON_SCHEDULER_SCHEDULER_H_

#include <unistd.h>
#include <atomic>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>

#include "cybertron/common/log.h"
#include "cybertron/common/macros.h"
#include "cybertron/common/types.h"
#include "cybertron/croutine/croutine.h"
#include "cybertron/croutine/routine_factory.h"
#include "cybertron/proto/routine_conf.pb.h"
#include "cybertron/proto/scheduler_conf.pb.h"

namespace apollo {
namespace cybertron {
namespace data {
class DataVisitor;
}
namespace scheduler {

class ProcBalancer;
class RoutineBalancer;

using apollo::cybertron::proto::RoutineConf;
using apollo::cybertron::proto::SchedulerConf;
using croutine::CRoutine;
using croutine::RoutineFactory;
using data::DataVisitor;

class Scheduler {
 public:
  ~Scheduler();
  bool CreateTask(const RoutineFactory& factory, const std::string& name);
  bool CreateTask(std::function<void()>&& func, const std::string& name,
                  std::shared_ptr<DataVisitor> visitor = nullptr);
  bool RemoveTask(const std::string& name);
  void PrintStatistics();
  void ShutDown();
  static uint32_t ProcessorNum() { return processor_num_; }

 private:
  Scheduler(Scheduler&) = delete;
  Scheduler& operator=(Scheduler&) = delete;

  void StartSourceBalance();

  uint64_t duration_ = 500000;
  std::atomic<bool> stop_;

  std::thread schedule_thread_;

  SchedulerConf scheduler_conf_;
  RoutineConf routine_conf_;

  std::shared_ptr<ProcBalancer> proc_balancer_;
  std::shared_ptr<RoutineBalancer> routine_balancer_;

  std::mutex task_id_map_mutex_;
  std::unordered_map<uint64_t, std::string> task_id_map_;

  static uint32_t processor_num_;

  DECLARE_SINGLETON(Scheduler)
};

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_SCHEDULER_SCHEDULER_H_
