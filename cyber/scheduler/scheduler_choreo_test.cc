/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <algorithm>
#include "gtest/gtest.h"

#include "cyber/common/global_data.h"
#include "cyber/cyber.h"
#include "cyber/scheduler/policy/choreography_context.h"
#include "cyber/scheduler/policy/classic_context.h"
#include "cyber/scheduler/policy/scheduler_choreography.h"
#include "cyber/scheduler/processor.h"
#include "cyber/scheduler/scheduler_factory.h"

namespace apollo {
namespace cyber {
namespace scheduler {

void func() {}

TEST(SchedulerChoreoTest, choreo) {
  auto processor = std::make_shared<Processor>();
  auto ctx = std::make_shared<ChoreographyContext>();
  processor->BindContext(ctx);

  std::shared_ptr<CRoutine> cr = std::make_shared<CRoutine>(func);
  auto task_id = GlobalData::RegisterTaskName("choreo");
  cr->set_id(task_id);
  EXPECT_TRUE(static_cast<ChoreographyContext*>(ctx.get())->Enqueue(cr));
  ctx->Shutdown();
}

TEST(SchedulerChoreoTest, sched_choreo) {
  GlobalData::Instance()->SetProcessGroup("example_sched_choreography");
  auto sched = dynamic_cast<SchedulerChoreography*>(scheduler::Instance());
  cyber::Init("SchedulerChoreoTest");
  std::shared_ptr<CRoutine> cr = std::make_shared<CRoutine>(func);
  cr->set_id(GlobalData::RegisterTaskName("sched_choreo"));
  cr->set_name("sched_choreo");
  EXPECT_TRUE(sched->DispatchTask(cr));

  std::shared_ptr<CRoutine> cr1 = std::make_shared<CRoutine>(func);
  cr1->set_id(GlobalData::RegisterTaskName("sched_choreo1"));
  cr1->set_name("sched_choreo1");
  cr1->set_processor_id(0);
  EXPECT_TRUE(sched->DispatchTask(cr1));

  auto& croutines =
      ClassicContext::cr_group_[DEFAULT_GROUP_NAME].at(cr->priority());
  std::vector<std::string> cr_names;
  for (auto& croutine : croutines) {
    cr_names.emplace_back(croutine->name());
  }
  auto itr = std::find(cr_names.begin(), cr_names.end(), cr->name());
  EXPECT_NE(itr, cr_names.end());

  itr = std::find(cr_names.begin(), cr_names.end(), cr1->name());
  EXPECT_EQ(itr, cr_names.end());

  sched->RemoveTask(cr->name());
  croutines = ClassicContext::cr_group_[DEFAULT_GROUP_NAME].at(cr->priority());
  cr_names.clear();
  for (auto& croutine : croutines) {
    cr_names.emplace_back(croutine->name());
  }
  itr = std::find(cr_names.begin(), cr_names.end(), cr->name());
  EXPECT_EQ(itr, cr_names.end());
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
