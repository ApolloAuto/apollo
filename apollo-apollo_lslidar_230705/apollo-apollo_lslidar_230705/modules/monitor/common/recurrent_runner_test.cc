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

#include "modules/monitor/common/recurrent_runner.h"

#include "cyber/common/log.h"
#include "gtest/gtest.h"

namespace apollo {
namespace monitor {

class DummyRecurrentRunner : public RecurrentRunner {
 public:
  explicit DummyRecurrentRunner(const double interval)
      : RecurrentRunner("DummyRecurrentRunner", interval) {}

  void RunOnce(const double current_time) override {}

  inline unsigned int GetRoundCount() { return round_count_; }
};

TEST(RecurrentRunnerTest, Tick) {
  DummyRecurrentRunner runner(1);
  EXPECT_EQ(0, runner.GetRoundCount());

  runner.Tick(0.1);  // Triggered. Next trigger time: 1.1
  EXPECT_EQ(1, runner.GetRoundCount());

  runner.Tick(1.0);  // Skipped. Next trigger time: 1.1
  EXPECT_EQ(1, runner.GetRoundCount());

  runner.Tick(2);    // Triggered. Next trigger time: 3
  runner.Tick(2.9);  // Not triggered. Next trigger time: 3
  runner.Tick(9);    // Triggered. Next trigger time: 10
  EXPECT_EQ(3, runner.GetRoundCount());
}

}  // namespace monitor
}  // namespace apollo
