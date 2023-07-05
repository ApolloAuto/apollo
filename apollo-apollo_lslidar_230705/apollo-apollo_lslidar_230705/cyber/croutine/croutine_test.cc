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
#include "cyber/croutine/croutine.h"

#include "gtest/gtest.h"

#include "cyber/common/global_data.h"
#include "cyber/cyber.h"
#include "cyber/init.h"

namespace apollo {
namespace cyber {
namespace croutine {

void function() { CRoutine::Yield(RoutineState::IO_WAIT); }

TEST(Croutine, croutinetest) {
  apollo::cyber::Init("croutine_test");
  std::shared_ptr<CRoutine> cr = std::make_shared<CRoutine>(function);
  auto id = GlobalData::RegisterTaskName("croutine");
  cr->set_id(id);
  cr->set_name("croutine");
  cr->set_processor_id(0);
  cr->set_priority(1);
  cr->set_state(RoutineState::DATA_WAIT);
  EXPECT_EQ(cr->state(), RoutineState::DATA_WAIT);
  cr->Wake();
  EXPECT_EQ(cr->state(), RoutineState::READY);
  cr->UpdateState();
  EXPECT_EQ(cr->state(), RoutineState::READY);
  EXPECT_EQ(*(cr->GetMainStack()), nullptr);
  cr->Resume();
  EXPECT_NE(*(cr->GetMainStack()), nullptr);
  EXPECT_EQ(cr->state(), RoutineState::IO_WAIT);
  cr->Stop();
  EXPECT_EQ(cr->Resume(), RoutineState::FINISHED);
}

}  // namespace croutine
}  // namespace cyber
}  // namespace apollo
