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

#include "cybertron/scheduler/proc_balancer.h"

#include "cybertron/common/global_data.h"
#include "cybertron/croutine/croutine.h"
#include "cybertron/scheduler/policy/cfs_context.h"
#include "cybertron/scheduler/policy/fcfs_context.h"
#include "cybertron/scheduler/policy/processor_context.h"
#include "cybertron/scheduler/policy/rq_context.h"
#include "cybertron/scheduler/scheduler.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

using apollo::cybertron::common::GlobalData;
using apollo::cybertron::croutine::CRoutine;

ProcBalancer::ProcBalancer() {
  auto global_conf = common::GlobalData::Instance()->Config();
  if (global_conf.has_scheduler_conf()) {
    proto::SchedulerConf scheduler_conf;
    scheduler_conf.CopyFrom(global_conf.scheduler_conf());
    strategy_ = scheduler_conf.process_strategy();
    enable_emergency_thread_ = scheduler_conf.buffer_processor();
  }

  for (int cpu_index = 0; cpu_index < Scheduler::ProcessorNum(); cpu_index++) {
    auto processor = std::make_shared<Processor>(enable_emergency_thread_);
    std::shared_ptr<ProcessorContext> processor_context;
    switch (strategy_) {
      case proto::ProcessStrategy::CFS:
        processor_context.reset(new CFSContext(processor));
        break;
      case proto::ProcessStrategy::RQ:
        processor_context.reset(new RQContext(processor));
        break;
      case proto::ProcessStrategy::FCFS:
        processor_context.reset(new FCFSContext(processor));
        break;
      default:
        processor_context.reset(new RQContext(processor));
        break;
    }
    processor_context->SetId(cpu_index);
    processor->SetId(cpu_index);
    processor->BindContext(processor_context);
    processor->Start();
    processor_list_.emplace_back(processor);
  }
}

bool ProcBalancer::Push(uint32_t processor_id,
                        const std::weak_ptr<CRoutine>& croutine) {
  auto cr = croutine.lock();
  if (!cr) {
    return false;
  }
  if (processor_id >= Scheduler::ProcessorNum()) {
    return false;
  }
  processor_list_[processor_id]->Context()->Push(cr);
  croutine_pos_map_.Set(cr->Id(), processor_id);
  AINFO << "push routine[" << GlobalData::GetTaskNameById(cr->Id())
        << "] into processor index[" << processor_id << "]";
  return true;
}

bool ProcBalancer::Push(const std::weak_ptr<CRoutine>& croutine) {
  auto cr = croutine.lock();
  if (!cr) {
    return false;
  }
  auto processor = GetProperProcessor(cr);
  cr->SetProcessorId(processor->Id());
  processor->Context()->Push(cr);
  croutine_pos_map_.Set(cr->Id(), processor->Id());
  AINFO << "push routine[" << GlobalData::GetTaskNameById(cr->Id())
        << "] into processor[" << processor->Id() << "]";
  return true;
}

std::shared_ptr<Processor> ProcBalancer::GetProperProcessor(
    const std::weak_ptr<CRoutine>& croutine) {
  auto cr = croutine.lock();
  if (overall_proc_stat_.exec_time <= 0.000001 &&
      overall_proc_stat_.sleep_time <= 0.000001) {
    return processor_list_[cr->ProcessorId()];
  }
  double min_ratio = -1;
  ProcessorStat proc_stat;
  std::shared_ptr<Processor> processor;
  for (int i = 0; i < processor_list_.size(); ++i) {
    auto pc = processor_list_[i];
    proc_stat = pc->Stat();
    double ratio = proc_stat.exec_time / overall_proc_stat_.exec_time +
                   proc_stat.sleep_time / overall_proc_stat_.sleep_time;
    if (min_ratio < 0) {
      min_ratio = ratio;
      processor = pc;
      continue;
    }
    if (ratio < min_ratio) {
      min_ratio = ratio;
      processor = pc;
    }
  }
  return processor;
}

bool ProcBalancer::GetProcessor(uint64_t routine_id, uint32_t* processor_id) {
  return croutine_pos_map_.Get(routine_id, processor_id);
}

bool ProcBalancer::RemoveCRoutine(uint64_t routine_id) {
  uint32_t id = 0;
  if (croutine_pos_map_.Get(routine_id, &id)) {
    croutine_pos_map_.Remove(routine_id);
    processor_list_[id]->Context()->RemoveCRoutine(routine_id);
    return true;
  }
  return false;
}

bool ProcBalancer::NotifyProcessor(uint64_t routine_id) {
  uint32_t id = 0;
  if (croutine_pos_map_.Get(routine_id, &id)) {
    processor_list_[id]->Context()->NotifyProcessor(routine_id);
    return true;
  }
  return false;
}

bool ProcBalancer::Run() {
  overall_proc_stat_.exec_time = 0;
  overall_proc_stat_.sleep_time = 0;
  ProcessorStat proc_stat;
  for (auto& processor : processor_list_) {
    processor->UpdateStat(&proc_stat);
    overall_proc_stat_.exec_time += proc_stat.exec_time;
    overall_proc_stat_.sleep_time += proc_stat.sleep_time;
  }
  return WorkStealing();
}

bool ProcBalancer::WorkStealing() { return false; }

void ProcBalancer::PrintStatistics() {
  AINFO << "********** Processor Statisitcs **********" << std::endl;
  for (auto& processor : processor_list_) {
    processor->Context()->PrintStatistics();
  }

  int index = 0;
  for (auto& processor : processor_list_) {
    AINFO << "********** Routine Statistics processor_index[" << index++
          << "] **********" << std::endl;
    processor->Context()->PrintCRoutineStats();
  }
}

void ProcBalancer::ShutDown() {
  for (auto& processor : processor_list_) {
    processor->Context()->ShutDown();
    processor.reset();
  }
  processor_list_.clear();
}

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo
