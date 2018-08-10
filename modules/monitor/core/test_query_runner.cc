/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include <thread>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "modules/monitor/sysmon/core/query_runner.h"

namespace apollo {
namespace monitor {
namespace sysmon {

class DummyTask {
public:
  MOCK_METHOD0(run, int());
};


TEST(PeriodicTaskTest, RunTest) {
  DummyTask dt1;
  DummyTask dt2;
  std::list<QueryTask> tasks;
  tasks.push_back(std::bind(&DummyTask::run, &dt1));
  tasks.push_back(std::bind(&DummyTask::run, &dt2));

  // Set up a runner that runs every 1 second.
  PeriodicQueryRunner mqr(1*NANOS_IN_SEC, std::move(tasks));

  EXPECT_CALL(dt1, run()).Times(3);
  EXPECT_CALL(dt2, run()).Times(3);
  std::thread tr(mqr.get_runner());

  // Sleep a little bit over 2 seconds then stop the runner, query tasks are expected to run 3 times
  // (tasks will be run once at the beginning, then once every second.
  std::this_thread::sleep_for(std::chrono::milliseconds(2030));
  mqr.stop();

  tr.join();
}

TEST(PeriodicQueryThreadTest, DoTest) {
  DummyTask dt1;
  DummyTask dt2;
  std::list<QueryTask> tasks;
  tasks.push_back(std::bind(&DummyTask::run, &dt1));
  tasks.push_back(std::bind(&DummyTask::run, &dt2));

  EXPECT_CALL(dt1, run()).Times(3);
  EXPECT_CALL(dt2, run()).Times(3);

  // Set up a thread that runs queries every 1 second.
  PeriodicQueryThread<PeriodicQueryRunner> mqr(1*NANOS_IN_SEC, std::move(tasks));
  mqr.start();

  // Sleep a little bit over 2 seconds then stop the runner, query tasks are expected to run 3 times
  // (tasks will be run once at the beginning, then once every second.
  std::this_thread::sleep_for(std::chrono::milliseconds(2030));

  mqr.stop();
  mqr.wait();
}

TEST(QueryOnceTest, DoTest) {
  DummyTask dt1;
  DummyTask dt2;
  std::list<QueryTask> tasks;
  tasks.push_back(std::bind(&DummyTask::run, &dt1));
  tasks.push_back(std::bind(&DummyTask::run, &dt2));

  EXPECT_CALL(dt1, run()).Times(1);
  EXPECT_CALL(dt2, run()).Times(1);

  QueryOnceThread qt(std::move(tasks));
  qt.start();
  qt.wait();
}

}  // namespace sysmon
}  // namespace monitor
}  // namespace apollo
