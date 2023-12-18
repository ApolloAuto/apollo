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
#include "gtest/gtest.h"

#include "cyber/common/log.h"

#include "modules/perception/common/lib/thread/concurrent_queue.h"
#include "modules/perception/common/lib/thread/thread_pool.h"

namespace apollo {
namespace perception {
namespace lib {

// Just use the protobuf Closure.
using google::protobuf::Closure;
using google::protobuf::NewCallback;

using std::vector;

void Callback(int tag, BlockingCounter* counter) {
  AINFO << "callback executed, tag: " << tag;
  counter->Decrement();
}

class MyCallback {
 public:
  void Callback(int tag, BlockingCounter* counter) {
    AINFO << "MyCallback::callback executed, tag: " << tag;
    counter->Decrement();
  }
};

TEST(ThreadPoolTest, Test) {
  ThreadPool thread_pool(2);
  thread_pool.Start();

  BlockingCounter counter(10);
  for (int idx = 0; idx < 10; ++idx) {
    Closure* closure = NewCallback(&Callback, idx, &counter);
    thread_pool.Add(closure);
  }

  counter.Wait();
  EXPECT_EQ(thread_pool.num_workers(), 2);
  EXPECT_EQ(thread_pool.num_available_workers(), 2);

  MyCallback my_callback;
  counter.Reset(10);
  for (int idx = 0; idx < 10; ++idx) {
    Closure* closure =
        NewCallback(&my_callback, &MyCallback::Callback, idx, &counter);
    thread_pool.Add(closure);
  }

  counter.Wait();

  counter.Reset(10);
  vector<Closure*> closures;
  for (int idx = 0; idx < 10; ++idx) {
    Closure* closure =
        NewCallback(&my_callback, &MyCallback::Callback, idx, &counter);
    closures.push_back(closure);
  }
  thread_pool.Add(closures);
  counter.Wait();

  EXPECT_EQ(thread_pool.num_workers(), 2);
  EXPECT_EQ(thread_pool.num_available_workers(), 2);
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo
