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

#include "cyber/scheduler/scheduler.h"

#include <string>
#include <thread>
#include "gtest/gtest.h"

#include "cyber/common/global_data.h"
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/proto/scheduler_conf.pb.h"
#include "cyber/scheduler/processor_context.h"
#include "cyber/scheduler/scheduler_factory.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::proto::InnerThread;

void proc() {}

TEST(SchedulerTest, create_task) {
  GlobalData::Instance()->SetProcessGroup("example_sched_classic");
  auto sched = Instance();
  cyber::Init("scheduler_test");
  // read example_sched_classic.conf task 'ABC' prio
  std::string croutine_name = "ABC";

  EXPECT_TRUE(sched->CreateTask(&proc, croutine_name));
  // create a croutine with the same name
  EXPECT_FALSE(sched->CreateTask(&proc, croutine_name));

  auto task_id = GlobalData::RegisterTaskName(croutine_name);
  EXPECT_TRUE(sched->NotifyTask(task_id));
  EXPECT_TRUE(sched->RemoveTask(croutine_name));
  // remove the same task twice
  EXPECT_FALSE(sched->RemoveTask(croutine_name));
  // remove a not exist task
  EXPECT_FALSE(sched->RemoveTask("driver"));
}

TEST(SchedulerTest, notify_task) {
  auto sched = Instance();
  cyber::Init("scheduler_test");
  std::string name = "croutine";
  auto id = GlobalData::RegisterTaskName(name);
  // notify task that the id is not exist
  EXPECT_FALSE(sched->NotifyTask(id));
  EXPECT_TRUE(sched->CreateTask(&proc, name));
  EXPECT_TRUE(sched->NotifyTask(id));
}

TEST(SchedulerTest, set_inner_thread_attr) {
  auto sched = Instance();
  cyber::Init("scheduler_test");
  std::thread t = std::thread([]() {});
  std::unordered_map<std::string, InnerThread> thread_confs;
  InnerThread inner_thread;
  inner_thread.set_cpuset("0-1");
  inner_thread.set_policy("SCHED_FIFO");
  inner_thread.set_prio(10);
  thread_confs["inner_thread_test"] = inner_thread;
  sched->SetInnerThreadConfs(thread_confs);
  sched->SetInnerThreadAttr("inner_thread_test", &t);
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  if (t.joinable()) {
    t.join();
  }
  sched->Shutdown();
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
