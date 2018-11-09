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

#include "cyber/scheduler/policy/scheduler_choreography.h"

#include <memory>

#include "cyber/scheduler/processor.h"
#include "cyber/scheduler/policy/task_choreo.h"
#include "cyber/scheduler/policy/classic.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::common::GlobalData;

SchedulerChoreography::SchedulerChoreography() {
  sched_policy_ = SchedPolicy::CHOREO;

  auto gconf = GlobalData::Instance()->Config();
  for (auto& conf : gconf.scheduler_conf().confs()) {
    sched_confs_[conf.name()] = conf;
  }

  auto desc = SchedName_descriptor()->
      FindValueByName(GlobalData::Instance()->SchedName());
  int sname;
  if (desc) {
    sname = desc->number();

    SchedConf sconf;
    auto itr = sched_confs_.find(sname);
    if (itr != sched_confs_.end()) {
      proc_num_ = itr->second.proc_num();
      task_pool_size_ = itr->second.task_pool_size();
      cpu_binding_start_index_ = itr->second.cpu_binding_start_index();
    }
  }

  for (auto& conf : gconf.choreo_conf()) {
    if (conf.sched_name() == sname) {
      for (auto& choreo : conf.choreos()) {
        cr_confs_[choreo.name()] = choreo;
      }
    }
  }

  // default val for case w/o config:
  if (proc_num_ == 0) {
    proc_num_ = std::thread::hardware_concurrency() / 4 * 3;
    // FIXME: other vals ... ?
  }
  // Currently for compatible with task/task_manager.cc,
  // will be deleted at last:
  // auto pool_size = scheduler::Scheduler::Instance()->TaskPoolSize();
  task_pool_size_ = std::thread::hardware_concurrency() / 4;

  CreateProcessor();
}

void SchedulerChoreography::CreateProcessor() {
  for (uint32_t i = 0; i < proc_num_; i++) {
    auto proc = std::make_shared<Processor>();
    auto ctx = std::make_shared<TaskChoreoContext>();

    proc->BindContext(ctx);
    proc->SetBindCpuIndex(cpu_binding_start_index_ + i);
    ctx->BindProc(proc);
    proc_ctxs_.emplace_back(ctx);

    proc->Start();
  }

  // For choreography policy: put tasks w/o processor assigned
  // to a classic pool.
  for (uint32_t i = 0; i < task_pool_size_; i++) {
    auto proc = std::make_shared<Processor>();
    auto ctx = std::make_shared<ClassicContext>();

    proc->BindContext(ctx);
    proc->SetBindCpuIndex(cpu_binding_start_index_ + proc_num_ + i);
    ctx->BindProc(proc);
    proc_ctxs_.emplace_back(ctx);

    classic_4_choreo_ = ctx.get();

    proc->Start();
  }
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo

