/**************Scheduler::****************************************************************
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

#include "cybertron/scheduler/scheduler.h"

#include "cybertron/common/global_data.h"
#include "cybertron/common/util.h"
#include "cybertron/data/data_visitor.h"
#include "cybertron/scheduler/proc_balancer.h"
#include "cybertron/scheduler/routine_balancer.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

using apollo::cybertron::proto::RoutineConfInfo;
using apollo::cybertron::common::GlobalData;

uint32_t Scheduler::processor_num_ = std::thread::hardware_concurrency();

Scheduler::Scheduler()
    : stop_(false),
      proc_balancer_(ProcBalancer::Instance()),
      routine_balancer_(RoutineBalancer::Instance()) {
  auto global_conf = GlobalData::Instance()->Config();
  if (global_conf.has_scheduler_conf()) {
    scheduler_conf_.CopyFrom(global_conf.scheduler_conf());
    if (scheduler_conf_.processor_num() == 0) {
      AWARN << "Processor num cannot be set to 0! For better performance, "
            << "will use the number of cpu cores as default value.";
    } else if (scheduler_conf_.processor_num() > processor_num_) {
      AWARN << "Processor num: " << scheduler_conf_.processor_num()
            << " is too large! For better performance, "
            << "will use the number of cpu cores as default value:"
            << processor_num_;
    } else {
      processor_num_ = scheduler_conf_.processor_num();
    }
  } else {
    AINFO << "No scheduler conf";
    return;
  }

  if (global_conf.has_routine_conf()) {
    routine_conf_.CopyFrom(global_conf.routine_conf());
  } else {
    AINFO << "No routine conf";
  }

  if (scheduler_conf_.load_balance()) {
    duration_ = scheduler_conf_.duration_us();
    StartSourceBalance();
  }
}

void Scheduler::ShutDown() {
  if (stop_.exchange(true)) {
    return;
  }

  if (schedule_thread_.joinable()) {
    schedule_thread_.join();
  }

  routine_balancer_->ShutDown();
  proc_balancer_->ShutDown();
}

Scheduler::~Scheduler() {}

bool Scheduler::CreateTask(const RoutineFactory& factory,
                           const std::string& name) {
  return CreateTask(factory.create_routine(), name, factory.GetDataVisitor());
}

bool Scheduler::CreateTask(std::function<void()>&& func,
                           const std::string& name,
                           std::shared_ptr<DataVisitor> visitor) {
  if (stop_) {
    AERROR << "scheduler is stoped, cannot create task!";
    return false;
  }
  auto task_id = GlobalData::RegisterTaskName(name);
  {
    std::lock_guard<std::mutex> lock(task_id_map_mutex_);
    if (task_id_map_.find(task_id) != task_id_map_.end()) {
      AERROR << "Routine [" << name << "] has been exists";
      return false;
    }
    task_id_map_[task_id] = name;
  }

  if (visitor != nullptr) {
    visitor->RegisterCallback(
        [this, task_id]() { this->proc_balancer_->NotifyProcessor(task_id); });
  }

  RoutineConfInfo conf_info;
  for (const auto& routine_info : routine_conf_.routine_info()) {
    if (routine_info.routine_name() == name) {
      conf_info.CopyFrom(routine_info);
      break;
    }
  }

  auto croutine = std::make_shared<CRoutine>(func);
  croutine->SetId(task_id);
  croutine->SetPriority(conf_info.priority());
  croutine->SetFrequency(conf_info.frequency());

  if (conf_info.has_processor_index()) {
    // avoid conf_info.processor_index() great than processor_num_
    auto processor_id = conf_info.processor_index() % processor_num_;
    croutine->SetProcessorId(processor_id);
    proc_balancer_->Push(processor_id, croutine);
  } else {
    croutine->SetProcessorId(routine_balancer_->RoutineSize() % processor_num_);
    proc_balancer_->Push(croutine);
  }

  routine_balancer_->Push(croutine);
  return true;
}

// TODO(xinjiankang)
bool Scheduler::RemoveTask(const std::string& name) {
  if (stop_) {
    return true;
  }
  return proc_balancer_->RemoveCRoutine(common::Hash(name));
}

void Scheduler::StartSourceBalance() {
  schedule_thread_ = std::thread([=]() {
    while (!stop_) {
      usleep(duration_);
      if (!proc_balancer_->Run()) {
        routine_balancer_->Run();
      }
    }
  });
}

void Scheduler::PrintStatistics() {
  routine_balancer_->PrintStatistics();
  proc_balancer_->PrintStatistics();
}

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo
