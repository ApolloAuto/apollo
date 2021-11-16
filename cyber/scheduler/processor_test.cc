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
#include "cyber/scheduler/processor.h"

#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "cyber/cyber.h"
#include "cyber/scheduler/common/pin_thread.h"
#include "cyber/scheduler/policy/choreography_context.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using scheduler::ChoreographyContext;
using scheduler::Processor;

TEST(ProcessorTest, all) {
  auto proc = std::make_shared<Processor>();
  auto context = std::make_shared<ChoreographyContext>();
  proc->BindContext(context);
  std::string affinity = "1to1";
  std::vector<int> cpuset;
  cpuset.emplace_back(0);
  SetSchedAffinity(proc->Thread(), cpuset, affinity, 0);
  SetSchedPolicy(proc->Thread(), "SCHED_OTHER", 0, proc->Tid());

  auto proc1 = std::make_shared<Processor>();
  auto context1 = std::make_shared<ChoreographyContext>();
  proc1->BindContext(context1);
  affinity = "range";
  SetSchedAffinity(proc1->Thread(), cpuset, affinity, 0);
  SetSchedPolicy(proc1->Thread(), "SCHED_FIFO", 0, proc1->Tid());

  proc->Stop();
  proc1->Stop();
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
