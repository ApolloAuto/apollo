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
#include "cybertron/task/task.h"

static const uint8_t num_threads = 3;

using apollo::cybertron::proto::Driver;

struct Message {
  uint64_t msg_id;
  uint64_t task_id;
  std::string content;
};

int TaskProcessor(const Message& msg) {
  //apollo::cybertron::USleep(100000);
  usleep(100000);
  AINFO << "ID is: " << msg.msg_id;
  return msg.msg_id;
}

void AsyncDataProcessor() {
  std::vector<std::future<int>> results;
  for (int i = 0; i < 1000; i++) {
    Message msg;
    msg.msg_id = i;
    results.push_back(apollo::cybertron::Async(&TaskProcessor, msg));
  }

  for (auto& result: results) {
    AINFO << "Result is: " << result.get();
  }
}

void VoidTaskProcessor(const std::shared_ptr<Message>& msg) {
  ADEBUG << "Task Processor[" << msg->task_id
         << "] is running: " << msg->msg_id;
}

int main(int argc, char* argv[]) {
  apollo::cybertron::Init(argv[0]);
  auto ret = apollo::cybertron::Async(&AsyncDataProcessor);
  ret.get();
  return 0;
}
