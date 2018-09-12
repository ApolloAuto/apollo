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
    AERROR << "AsyncDataProcesor is running.";
    apollo::cybertron::USleep(5000000);
  }
}

void TaskProcessor(const std::shared_ptr<Message>& msg) {
  AERROR << "Task Processor[" << msg->task_id
         << "] is running: " << msg->msg_id;
  apollo::cybertron::USleep(100000);
}

int main(int argc, char* argv[]) {
  // Init
  apollo::cybertron::Init(argv[0]);
  // Task<> task0("async_data_processor", &AsyncDataProcessor);
  auto task0 = apollo::cybertron::CreateTask("async_data_processor",
                                             &AsyncDataProcessor);
  auto task1 = apollo::cybertron::CreateTask<Message>(
      "task_processor", &TaskProcessor, num_threads);

  // Run
  uint64_t i = 0;
  while (!apollo::cybertron::IsShutdown()) {
    for (int j = 0; j < num_threads; ++j) {
      auto msg = std::make_shared<Message>();
      msg->msg_id = i++;
      msg->task_id = j;
      task1->Execute(msg);
      apollo::cybertron::Yield();
      apollo::cybertron::USleep(10000);
    }
    task1->Wait();
  }
  AERROR << "All task are finished.";
  return 0;
}
