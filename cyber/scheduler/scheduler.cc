/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Vesched_infoon 2.0 (the "License");
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

#include "cyber/scheduler/scheduler.h"

#include <sched.h>
#include <utility>

#include "cyber/common/environment.h"
#include "cyber/common/file.h"
#include "cyber/common/global_data.h"
#include "cyber/common/util.h"
#include "cyber/data/data_visitor.h"
#include "cyber/event/perf_event_cache.h"
#include "cyber/scheduler/processor.h"
#include "cyber/scheduler/processor_context.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::common::GlobalData;

bool Scheduler::CreateTask(const RoutineFactory& factory,
                           const std::string& name) {
  return CreateTask(factory.create_routine(), name, factory.GetDataVisitor());
}

bool Scheduler::CreateTask(std::function<void()>&& func,
                           const std::string& name,
                           std::shared_ptr<DataVisitorBase> visitor) {
  if (unlikely(stop_.load())) {
    ADEBUG << "scheduler is stoped, cannot create task!";
    return false;
  }

  auto task_id = GlobalData::RegisterTaskName(name);

  auto cr = std::make_shared<CRoutine>(func);
  cr->set_id(task_id);
  cr->set_name(name);
  AINFO << "create croutine: " << name;

  if (!DispatchTask(cr)) {
    return false;
  }

  if (visitor != nullptr) {
    visitor->RegisterNotifyCallback([this, task_id, name]() {
      if (unlikely(stop_.load())) {
        return;
      }
      this->NotifyProcessor(task_id);
    });
  }
  return true;
}

bool Scheduler::NotifyTask(uint64_t crid) {
  if (unlikely(stop_.load())) {
    return true;
  }
  return NotifyProcessor(crid);
}

void Scheduler::ParseCpuset(const std::string& str, std::vector<int>* cpuset) {
  std::vector<std::string> lines;
  std::stringstream ss(str);
  std::string l;

  while (getline(ss, l, ',')) {
    lines.push_back(l);
  }

  for (auto line : lines) {
    std::stringstream ss(line);
    std::vector<std::string> range;

    while (getline(ss, l, '-')) {
      range.push_back(l);
    }

    if (range.size() == 1) {
      cpuset->push_back(std::stoi(range[0]));
    } else if (range.size() == 2) {
      for (int i = std::stoi(range[0]), e = std::stoi(range[1]); i <= e; i++) {
        cpuset->push_back(i);
      }
    } else {
      AERROR << "Parsing cpuset format error.";
      exit(0);
    }
  }
}

void Scheduler::ProcessLevelResourceControl() {
  std::vector<int> cpus;
  ParseCpuset(process_level_cpuset_, &cpus);
  cpu_set_t set;
  CPU_ZERO(&set);
  for (const auto cpu : cpus) {
    CPU_SET(cpu, &set);
  }
  pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
}

void Scheduler::SetInnerThreadAttr(const std::string& name, std::thread* thr) {
  if (thr != nullptr && inner_thr_confs_.find(name) != inner_thr_confs_.end()) {
    auto th_conf = inner_thr_confs_[name];
    auto cpuset = th_conf.cpuset();

    std::vector<int> cpus;
    ParseCpuset(cpuset, &cpus);
    cpu_set_t set;
    CPU_ZERO(&set);
    for (const auto cpu : cpus) {
      CPU_SET(cpu, &set);
    }
    pthread_setaffinity_np(thr->native_handle(), sizeof(set), &set);

    auto policy = th_conf.policy();
    auto prio = th_conf.prio();
    int p;
    if (!policy.compare("SCHED_FIFO")) {
      p = SCHED_FIFO;
    } else if (!policy.compare("SCHED_RR")) {
      p = SCHED_RR;
    } else {
      return;
    }

    struct sched_param sp;
    memset(static_cast<void*>(&sp), 0, sizeof(sp));
    sp.sched_priority = prio;
    pthread_setschedparam(thr->native_handle(), p, &sp);
  }
  return;
}

void Scheduler::Shutdown() {
  if (unlikely(stop_.exchange(true))) {
    return;
  }

  for (auto& ctx : pctxs_) {
    ctx->Shutdown();
  }

  std::vector<uint64_t> cr_list;
  {
    ReadLockGuard<AtomicRWLock> lk(id_cr_lock_);
    for (auto& cr : id_cr_) {
      cr_list.emplace_back(cr.second->id());
    }
  }

  for (auto& id : cr_list) {
    RemoveCRoutine(id);
  }

  for (auto& processor : processors_) {
    processor->Stop();
  }

  processors_.clear();
  pctxs_.clear();
}
}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
