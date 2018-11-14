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

#include <utility>

#include "cyber/common/global_data.h"
#include "cyber/common/util.h"
#include "cyber/data/data_visitor.h"
#include "cyber/event/perf_event_cache.h"
#include "cyber/scheduler/policy/choreography.h"
#include "cyber/scheduler/policy/classic.h"
#include "cyber/scheduler/policy/scheduler_choreography.h"
#include "cyber/scheduler/policy/scheduler_classic.h"
#include "cyber/scheduler/processor.h"
#include "cyber/scheduler/processor_context.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::common::GlobalData;
using apollo::cyber::event::PerfEventCache;
using apollo::cyber::event::SchedPerf;

Scheduler* Scheduler::Instance() {
  static Scheduler* instance = nullptr;

  if (unlikely(!instance)) {
    SchedPolicy spolicy = SchedPolicy::CLASSIC;

    // Get sched policy from conf
    auto gconf = GlobalData::Instance()->Config();
    std::unordered_map<int, SchedConf> sconfs;
    for (auto& conf : gconf.scheduler_conf().confs()) {
      sconfs[conf.name()] = conf;
    }

    auto desc = SchedName_descriptor()->FindValueByName(
        GlobalData::Instance()->SchedName());
    if (desc) {
      int sname = desc->number();
      auto itr = sconfs.find(sname);
      if (itr != sconfs.end()) {
        spolicy = itr->second.policy();
      }
    }

    switch (spolicy) {
      case SchedPolicy::CHOREO:
        instance = new SchedulerChoreography();
        break;
      case SchedPolicy::CLASSIC:
      default:
        instance = new SchedulerClassic();
        break;
    }
  }

  return instance;
}

void Scheduler::ShutDown() {
  if (stop_.exchange(true)) {
    return;
  }

  for (auto& ctx : pctxs_) {
    ctx->ShutDown();
  }
  pctxs_.clear();
}

bool Scheduler::CreateTask(const RoutineFactory& factory,
                           const std::string& name) {
  return CreateTask(factory.create_routine(), name, factory.GetDataVisitor());
}

bool Scheduler::CreateTask(std::function<void()>&& func,
                           const std::string& name,
                           std::shared_ptr<DataVisitorBase> visitor) {
  if (stop_) {
    AERROR << "scheduler is stoped, cannot create task!";
    return false;
  }

  auto task_id = GlobalData::RegisterTaskName(name);

  auto cr = std::make_shared<CRoutine>(func);
  cr->set_id(task_id);
  cr->set_name(name);

  Choreo conf;
  if (cr_confs_.find(name) != cr_confs_.end()) {
    conf = cr_confs_[name];
    cr->set_priority(conf.priority());

    if (conf.has_processor_index()) {
      auto proc_id = conf.processor_index();
      cr->set_processor_id(proc_id);
    }
  }

  if (!DispatchTask(cr)) {
    return false;
  }

  if (visitor != nullptr) {
    visitor->RegisterNotifyCallback([this, task_id, name]() {
      if (stop_) {
        return;
      }
      this->NotifyProcessor(task_id);
    });
  }
  return true;
}

bool Scheduler::NotifyTask(uint64_t crid) {
  if (stop_) {
    return true;
  }
  return NotifyProcessor(crid);
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
