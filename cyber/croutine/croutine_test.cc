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

#include <chrono>
#include <memory>

#include "cyber/croutine/croutine.h"
#include "cyber/croutine/routine_context.h"
#include "cyber/cyber.h"
#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace croutine {

void Sleep(uint64_t ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

class ParameterServerTest : public ::testing::Test {
  virtual void SetUp() {
    apollo::cyber::Init();
    auto context =
        std::make_shared<apollo::cyber::croutine::RoutineContext>();
    apollo::cyber::croutine::CRoutine::SetMainContext(context);
  }
};

}  // namespace croutine
}  // namespace cyber
}  // namespace apollo
