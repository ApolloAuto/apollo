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

#include "modules/common/util/threadpool.h"

#include <algorithm>
#include <atomic>
#include <iterator>
#include <set>
#include <sstream>
#include <string>

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace util {

namespace {

std::atomic<int> n(0);

void simple_add() { n++; }
void simple_minus(int k) { n -= k; }

}  // namespace

TEST(ThreadPool, simple) {
  ThreadPool p(5);
  std::vector<std::future<void>> k;
  for (int i = 0; i < 1000; ++i) {
    auto f1 = std::bind(simple_add);
    k.push_back(std::move(p.enqueue(f1)));
  }
  for (auto& task : k) {
    task.wait();
  }
  EXPECT_EQ(1000, n.load());

  k.clear();

  for (int i = 0; i < 500; ++i) {
    auto f1 = std::bind(simple_add);
    auto f2 = std::bind(simple_minus, 1);
    auto t1 = p.enqueue(f1);
    auto t2 = p.enqueue(f2);
    k.push_back(std::move(t1));
    k.push_back(std::move(t2));
  }
  for (auto& task : k) {
    task.wait();
  }
  EXPECT_EQ(1000, n.load());
}

}  // namespace util
}  // namespace common
}  // namespace apollo
