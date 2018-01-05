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

#include "modules/common/util/ctpl_stl.h"

#include <atomic>

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace util {

namespace {

std::atomic<int> n(0);

void simple_add() { n++; }
void simple_minus() { n--; }
}

TEST(ThreadPool, simple) {
  ThreadPool p(5);
  for (int i = 0; i < 1000; ++i) {
    auto f1 = std::bind(simple_add);
    p.push(f1);
  }
  p.stop(true);
  EXPECT_EQ(n.load(), 1000);

  for (int i = 0; i < 500; ++i) {
    auto f1 = std::bind(simple_add);
    auto f2 = std::bind(simple_minus);
    p.push(f1);
    p.push(f2);
  }
  p.stop(true);
  EXPECT_EQ(n.load(), 1000);
}

}  // namespace util
}  // namespace common
}  // namespace apollo
