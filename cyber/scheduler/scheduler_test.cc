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

#include <gtest/gtest.h>
#include <string>

#include "cyber/cyber.h"
#include "cyber/scheduler/scheduler.h"
#include "cyber/common/global_data.h"
#include "cyber/proto/scheduler_conf.pb.h"
#include "cyber/scheduler/processor_context.h"

namespace apollo {
namespace cyber {
namespace scheduler {

auto sched = Scheduler::Instance();

void proc() {}

TEST(SchedulerTest, create_task) {
  cyber::Init();
  std::string croutine_name = "DriverProc";

  EXPECT_TRUE(sched->CreateTask(&proc, croutine_name));
  // create a croutine with the same name
  EXPECT_FALSE(sched->CreateTask(&proc, croutine_name));
  auto task_id = GlobalData::RegisterTaskName(croutine_name);
  EXPECT_TRUE(sched->NotifyTask(task_id));
  EXPECT_TRUE(sched->RemoveTask(croutine_name));

  auto proc_num = std::thread::hardware_concurrency();
  auto gconf = GlobalData::Instance()->Config();
  apollo::cyber::proto::SchedulerConf sched_conf;
  sched_conf.CopyFrom(gconf.scheduler_conf());
  proc_num = sched_conf.processor_conf().processor_num();
  auto task_pool_size = sched_conf.task_pool_conf().task_pool_size();
  EXPECT_EQ(sched->ProcessorNum(), proc_num);
  EXPECT_EQ(sched->ProcCtxs().size(), proc_num + task_pool_size);

  std::shared_ptr<CRoutine> cr = std::make_shared<CRoutine>(proc);
  cr->set_id(task_id);
  cr->set_name(croutine_name);
  cr->set_processor_id(1);
  EXPECT_TRUE(sched->DispatchTask(cr));
  auto proc_ctxs = sched->ProcCtxs();
  RoutineState state;
  EXPECT_TRUE(proc_ctxs[1]->get_state(cr->id(), &state));

  sched->ShutDown();
  EXPECT_FALSE(sched->CreateTask(&proc, croutine_name));
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
