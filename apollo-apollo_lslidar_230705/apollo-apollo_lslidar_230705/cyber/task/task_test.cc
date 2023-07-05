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

#include "cyber/task/task.h"

#include <memory>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "cyber/init.h"

namespace apollo {
namespace cyber {
namespace scheduler {

class Foo {
 public:
  void RunOnce() {
    auto res = Async(&Foo::Task, this, 10);
    EXPECT_EQ(res.get(), 10);
  }

  uint32_t Task(const uint32_t& input) { return input; }
};

struct Message {
  uint64_t id;
};

void Task1() { ADEBUG << "Task1 running"; }

void Task2(const Message& input) {
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ADEBUG << "Task2 running";
}

uint64_t Task3(const std::shared_ptr<Message>& input) {
  ADEBUG << "Task3 running";
  return input->id;
}

TEST(AsyncTest, create_task) {
  auto task_1 = Async(&Task1);
  task_1.get();

  Message msg;
  auto task_2 = Async(&Task2, msg);
  task_2.get();

  auto shared_msg = std::make_shared<Message>();
  shared_msg->id = 1;
  auto task_3 = Async(&Task3, shared_msg);
  EXPECT_EQ(task_3.get(), 1);
}

TEST(AsyncTest, batch_run) {
  std::vector<std::future<void>> void_results;
  for (int i = 0; i < 10; i++) {
    void_results.push_back(Async(&Task1));
  }

  for (auto& result : void_results) {
    result.get();
  }

  int loop = 10;
  std::vector<std::future<uint64_t>> int_results;
  for (int i = 0; i < loop; i++) {
    auto shared_msg = std::make_shared<Message>();
    shared_msg->id = i;
    int_results.push_back(Async(&Task3, shared_msg));
  }

  for (int i = 0; i < loop; i++) {
    EXPECT_EQ(int_results[i].get(), i);
  }
}

TEST(AsyncTest, run_member_function) {
  Foo foo;
  foo.RunOnce();
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  apollo::cyber::Init(argv[0]);
  return RUN_ALL_TESTS();
}
