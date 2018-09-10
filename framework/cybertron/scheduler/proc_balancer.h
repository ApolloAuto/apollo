/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef CYBERTRON_SCHEDULER_PROC_BALANCER_H_
#define CYBERTRON_SCHEDULER_PROC_BALANCER_H_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include "cybertron/base/atomic_hash_map.h"
#include "cybertron/common/macros.h"
#include "cybertron/scheduler/processor.h"

namespace apollo {
namespace cybertron {
namespace croutineoutine {
  class croutineoutine;
}
namespace scheduler {

using apollo::cybertron::base::AtomicHashMap;

class ProcBalancer {
 public:
  ~ProcBalancer(){ ShutDown(); }

  std::shared_ptr<Processor> GetProperProcessor(
      const std::weak_ptr<CRoutine>& croutine);
  bool Push(const std::weak_ptr<CRoutine>& croutine);
  bool Push(uint32_t proc_id, const std::weak_ptr<CRoutine>& croutine);
  bool RemoveCRoutine(uint64_t id);
  bool NotifyProcessor(uint64_t croutine_id);
  bool GetProcessor(uint64_t croutine_id, uint32_t* processor_id);
  bool Run();
  void PrintStatistics();
  void ShutDown();

 private:
  bool WorkStealing();

  proto::ProcessStrategy strategy_ = proto::ProcessStrategy::FCFS;
  bool enable_emergency_thread_ = false;
  ProcessorStat overall_proc_stat_;
  std::vector<std::shared_ptr<Processor>> processor_list_;
  AtomicHashMap<uint64_t, uint32_t> croutine_pos_map_;
  DECLARE_SINGLETON(ProcBalancer);
};

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_SCHEDULER_PROC_BALANCER_H_
