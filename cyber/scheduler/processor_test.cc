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

#include <gtest/gtest.h>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "policy/choreography_context.h"

namespace apollo {
namespace cyber {
namespace processortest {

using scheduler::Processor;
using scheduler::ChoreographyContext;

TEST(ProcessorTest, all) {
  auto proc = std::make_shared<Processor>();
  auto context = std::make_shared<ChoreographyContext>();
  proc->BindContext(context);
  std::string affinity = "xxx";
  std::vector<int> cpuset;
  cpuset.emplace_back(0);
  proc->SetSchedAffinity(cpuset, affinity, 0);
  proc->SetSchedPolicy("SCHED_OTHER", 0);
  proc->Stop();
}

}  // namespace processortest
}  // namespace cyber
}  // namespace apollo
