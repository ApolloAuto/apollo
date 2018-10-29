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
}

TEST(SchedulerTest, async_task_create_notify) {
  cyber::Init();
  int task_num = 50;
  std::vector<std::string> names(task_num);
  std::vector<uint64_t> task_ids(task_num);
  for (int i = 0; i < task_num; i++) {
    names[i] = std::to_string(i);
    task_ids[i] = GlobalData::RegisterTaskName(std::to_string(i));
  }
  auto f = [&](std::string name) {
    sched->CreateTask(&proc, name);
  };
  auto f1 = [&](uint64_t id) {
    sched->NotifyTask(id);
  };
  std::vector<std::thread> task_create(task_num);
  std::vector<std::thread> task_notify(task_num);
  for (int i = 0; i < task_num; i++) {
    task_create[i] = std::thread(f, names[i]);
    task_notify[i] = std::thread(f1, task_ids[i]);
  }
  for (int i = 0; i < task_num; i++) {
    if (task_create[i].joinable()) {
      task_create[i].join();
    }
    if (task_notify[i].joinable()) {
      task_notify[i].join();
    }
  }
}


}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
