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

#ifndef CYBERTRON_SCHEDULER_ROUTINE_BALANCER_H_
#define CYBERTRON_SCHEDULER_ROUTINE_BALANCER_H_

#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>
#include "cybertron/common/macros.h"

namespace apollo {
namespace cybertron {
namespace croutine {
  class CRoutine;
}
namespace scheduler {

class ProcBalancer;

using croutine::CRoutine;

class RoutineBalancer {
 public:
  bool Push(const std::weak_ptr<CRoutine>& croutine);
  bool Run();
  void PrintStatistics();
  void ShutDown();
  uint32_t RoutineSize() { return croutine_list_.size(); }

 private:
  void Migrate(const std::weak_ptr<CRoutine>& croutine);

  std::vector<std::weak_ptr<CRoutine>> croutine_list_;
  std::set<uint64_t> rt_id_set_;
  std::shared_ptr<ProcBalancer> proc_balancer_ = nullptr;
  std::mutex rt_list_mutex_;
  std::mutex rt_name_mutex_;

  DECLARE_SINGLETON(RoutineBalancer);
};

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_SCHEDULER_ROUTINE_BALANCER_H_
