/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <iostream>
#include <memory>

#include "cybertron/common/log.h"
#include "cybertron/cybertron.h"
#include "cybertron/proto/driver.pb.h"
#include "cybertron/scheduler/task.h"

static const uint8_t num_threads = 3;

using apollo::cybertron::Task;
using apollo::cybertron::proto::Driver;

struct Message {
  uint64_t msg_id;
  uint64_t task_id;
  std::string content;
};

void AsyncDataProcessor() {
  for (;;) {
    AINFO << "AsyncDataProcesor is running.";
    apollo::cybertron::USleep(5000000);
  }
}

int TaskProcessor(const std::shared_ptr<Message>& msg) {
  apollo::cybertron::USleep(100000);
  return msg->msg_id;
}

void VoidTaskProcessor(const std::shared_ptr<Message>& msg) {
  AINFO << "Task Processor[" << msg->task_id
         << "] is running: " << msg->msg_id;
}

int main(int argc, char* argv[]) {
  // Init
  apollo::cybertron::Init(argv[0]);
  Task<> task0("async_data_processor", &AsyncDataProcessor, 1);

  Task<Message, int> task1(
      "task_processor", &TaskProcessor, num_threads);

  Task<Message, void> task2(
      "void_task_processor", &VoidTaskProcessor, num_threads);

  // Run
  uint64_t i = 0;
  while (!apollo::cybertron::IsShutdown()) {
    std::vector<std::future<int>> futures;
    for (int j = 0; j < num_threads; ++j) {
      auto msg = std::make_shared<Message>();
      msg->msg_id = i++;
      msg->task_id = j;
      futures.emplace_back(task1.Execute(msg));
      task2.Execute(msg);
    }

    apollo::cybertron::USleep(500000);
    if (apollo::cybertron::IsShutdown()) {
      break;
    }
    for (auto& future: futures) {
      AINFO << "Finish task:" << future.get();
    }
    AINFO << "All task are finished.";
  }
  AINFO << "Return";
  return 0;
}
