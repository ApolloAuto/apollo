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

#include "cybertron/scheduler/routine_balancer.h"

#include <future>

#include "cybertron/common/global_data.h"
#include "cybertron/scheduler/policy/processor_context.h"
#include "cybertron/scheduler/proc_balancer.h"
#include "cybertron/scheduler/processor.h"
#include "cybertron/scheduler/scheduler.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

using apollo::cybertron::common::GlobalData;

RoutineBalancer::RoutineBalancer() : proc_balancer_(ProcBalancer::Instance()) {}

bool RoutineBalancer::Push(const std::weak_ptr<CRoutine>& croutine) {
  std::lock_guard<std::mutex> lk(rt_list_mutex_);
  auto cr = croutine.lock();
  if (rt_id_set_.find(cr->Id()) != rt_id_set_.end()) {
    return false;
  }
  croutine_list_.emplace_back(cr);
  rt_id_set_.insert(cr->Id());
  return true;
}

bool RoutineBalancer::Run() {
  /*
  double ratio = 0.0;
  double max_ratio = -1;
  DataStatistics data_stat;
  RoutineStatistics routine_stat;
  std::shared_ptr<Routine> routine = nullptr;
  for (auto& rt : routine_list_) {
    rt->UpdateRoutineStat(&routine_stat);
    rt->UpdateDataStat(&data_stat);
    double drop_num = data_stat.drop_num;
    double pop_num = data_stat.pop_num;
    if (drop_num == 0 && pop_num == 0) {
      continue;
    }
    ratio = drop_num / (drop_num + pop_num);
    if (ratio < 0.75) {
      continue;
    }
    if (max_ratio < 0) {
      max_ratio = ratio;
      routine = rt;
      continue;
    }
    if (ratio > max_ratio) {
      max_ratio = ratio;
      routine = rt;
    }
  }
  if (routine != nullptr) {
    Migrate(routine);
  }
  */
  return true;
}

void RoutineBalancer::Migrate(const std::weak_ptr<CRoutine>& croutine) {
  /*
  uint32_t cur_proc_id = 0;
  if (!proc_balancer_->GetProcessor(rt->Id(), &cur_proc_id)) {
    return;
  }
  auto processor = proc_balancer_->GetProperProcessor(rt);
  if (cur_proc_id == processor->Id()) {
    return;
  }
  std::future<std::shared_ptr<CRoutine>> fut;
  if (processor->Context()->Pop(rt->Id(), fut)) {
    auto cr = fut.get();
    rt->SetCRoutine(cr);
    AINFO << "migrate routine[" << GlobalData::GetTaskNameById(rt->Id())
          << "] from_index[" << cur_proc_id << "] to_index[" << processor->Id()
          << "]";
    proc_balancer_->Push(processor->Id(), rt);
  }
  */
}

void RoutineBalancer::PrintStatistics() {
  return;
}

void RoutineBalancer::ShutDown() {}

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo
