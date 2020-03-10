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

#include "gtest/gtest.h"

#include "cyber/base/for_each.h"
#include "cyber/common/global_data.h"
#include "cyber/cyber.h"
#include "cyber/scheduler/policy/classic_context.h"
#include "cyber/scheduler/policy/scheduler_classic.h"
#include "cyber/scheduler/processor.h"
#include "cyber/scheduler/scheduler_factory.h"
#include "cyber/task/task.h"

namespace apollo {
namespace cyber {
namespace scheduler {

void func() {}

TEST(SchedulerClassicTest, classic) {
  auto processor = std::make_shared<Processor>();
  auto ctx = std::make_shared<ClassicContext>();
  processor->BindContext(ctx);
  std::vector<std::future<void>> res;

  // test single routine
  auto future = Async([]() {
    FOR_EACH(i, 0, 20) { cyber::SleepFor(std::chrono::milliseconds(i)); }
    AINFO << "Finish task: single";
  });
  future.get();

  // test multiple routine
  FOR_EACH(i, 0, 20) {
    res.emplace_back(Async([i]() {
      FOR_EACH(time, 0, 30) { cyber::SleepFor(std::chrono::milliseconds(i)); }
    }));
    AINFO << "Finish task: " << i;
  };
  for (auto& future : res) {
    future.wait_for(std::chrono::milliseconds(1000));
  }
  res.clear();
  ctx->Shutdown();
  processor->Stop();
}

TEST(SchedulerClassicTest, sched_classic) {
  // read example_sched_classic.conf
  GlobalData::Instance()->SetProcessGroup("example_sched_classic");
  auto sched1 = dynamic_cast<SchedulerClassic*>(scheduler::Instance());
  std::shared_ptr<CRoutine> cr = std::make_shared<CRoutine>(func);
  auto task_id = GlobalData::RegisterTaskName("ABC");
  cr->set_id(task_id);
  cr->set_name("ABC");
  EXPECT_TRUE(sched1->DispatchTask(cr));
  // dispatch the same task
  EXPECT_FALSE(sched1->DispatchTask(cr));
  EXPECT_TRUE(sched1->RemoveTask("ABC"));

  std::shared_ptr<CRoutine> cr1 = std::make_shared<CRoutine>(func);
  cr1->set_id(GlobalData::RegisterTaskName("xxxxxx"));
  cr1->set_name("xxxxxx");
  EXPECT_TRUE(sched1->DispatchTask(cr1));

  auto t = std::thread(func);
  sched1->SetInnerThreadAttr("shm", &t);
  if (t.joinable()) {
    t.join();
  }

  sched1->Shutdown();

  GlobalData::Instance()->SetProcessGroup("not_exist_sched");
  auto sched2 = dynamic_cast<SchedulerClassic*>(scheduler::Instance());
  std::shared_ptr<CRoutine> cr2 = std::make_shared<CRoutine>(func);
  cr2->set_id(GlobalData::RegisterTaskName("sched2"));
  cr2->set_name("sched2");
  EXPECT_TRUE(sched2->DispatchTask(cr2));
  sched2->Shutdown();

  GlobalData::Instance()->SetProcessGroup("dreamview_sched");
  auto sched3 = dynamic_cast<SchedulerClassic*>(scheduler::Instance());
  std::shared_ptr<CRoutine> cr3 = std::make_shared<CRoutine>(func);
  cr3->set_id(GlobalData::RegisterTaskName("sched3"));
  cr3->set_name("sched3");
  EXPECT_TRUE(sched3->DispatchTask(cr3));
  sched3->Shutdown();
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  apollo::cyber::Init(argv[0]);
  auto res = RUN_ALL_TESTS();
  apollo::cyber::Clear();
  return res;
}
