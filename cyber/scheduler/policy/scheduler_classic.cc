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

#include "cyber/scheduler/policy/scheduler_classic.h"

#include <memory>

#include "cyber/scheduler/processor.h"
#include "cyber/scheduler/policy/classic.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::common::GlobalData;

SchedulerClassic::SchedulerClassic() {
  sched_policy_ = SchedPolicy::CLASSIC;

  auto gconf = GlobalData::Instance()->Config();
  for (auto& conf : gconf.scheduler_conf().confs()) {
    sched_confs_[conf.name()] = conf;
  }

  auto desc = SchedName_descriptor()->
      FindValueByName(GlobalData::Instance()->SchedName());
  if (desc) {
    int sname = desc->number();
    SchedConf sconf;
    auto itr = sched_confs_.find(sname);
    if (itr != sched_confs_.end()) {
      proc_num_ = itr->second.proc_num();
      // just for compatible
      task_pool_size_ = proc_num_;
      cpu_binding_start_index_ = itr->second.cpu_binding_start_index();
    }
  }

  // default val for case w/o config:
  if (proc_num_ == 0) {
    proc_num_ = std::thread::hardware_concurrency();
    // FIXME: other vals ... ?
    proc_num_ = 2;
  }
  // Currently for compatible with task/task_manager.cc,
  // will be deleted at last:
  // auto pool_size = scheduler::Scheduler::Instance()->TaskPoolSize();
  task_pool_size_ = proc_num_;

  CreateProcessor();
}

void SchedulerClassic::CreateProcessor() {
  for (uint32_t i = 0; i < proc_num_; i++) {
    auto proc = std::make_shared<Processor>();
    auto ctx = std::make_shared<ClassicContext>();

    proc->BindContext(ctx);
    proc->SetBindCpuIndex(cpu_binding_start_index_ + i);
    ctx->BindProc(proc);
    proc_ctxs_.emplace_back(ctx);

    proc->Start();
  }
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo


