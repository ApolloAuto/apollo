/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/common/prediction_thread_pool.h"

#include <chrono>

#include "gtest/gtest.h"

namespace apollo {
namespace prediction {

TEST(PredictionThreadPoolTest, post_future) {
  BaseThreadPool pool(5, 0);
  int n = 1;
  std::future<int> r1 = pool.Post([&] { return n; });
  std::this_thread::sleep_for(std::chrono::microseconds(1000));

  std::future<int> r2 = pool.Post([&] {
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    return n;
  });

  n = 2;
  r1.get();
  EXPECT_EQ(2, r2.get());
}

TEST(PredictionThreadPoolTest, for_each) {
  BaseThreadPool pool(5, 0);
  std::vector<int> expect = {1, 2, 3, 4, 5, 6, 7, 8};
  std::vector<int> real = {1, 2, 3, 4, 5, 6, 7, 8};

  auto incr = [](int& input) { ++input; };

  std::for_each(expect.begin(), expect.end(), incr);
  pool.ForEach(real.begin(), real.end(), incr);

  EXPECT_EQ(expect, real);
}

TEST(PredictionThreadPoolTest, global_for_each) {
  std::vector<int> expect = {1, 2, 3, 4, 5, 6, 7, 8};
  std::vector<int> real = {1, 2, 3, 4, 5, 6, 7, 8};

  auto incr = [](int& input) { ++input; };

  std::for_each(expect.begin(), expect.end(), incr);
  PredictionThreadPool::ForEach(real.begin(), real.end(), incr);

  EXPECT_EQ(expect, real);
}

TEST(PredictionThreadPoolTest, avoid_deadlock) {
  std::vector<int> expect = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
  std::vector<int> real = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

  std::for_each(expect.begin(), expect.end(), [](int& input) {
    std::vector<int> vec = {1, 2, 3, 4};
    std::for_each(vec.begin(), vec.end(), [](int& v) { ++v; });
    input = std::accumulate(vec.begin(), vec.end(), input);
  });

  EXPECT_EQ(0, PredictionThreadPool::s_thread_pool_level);

  PredictionThreadPool::ForEach(real.begin(), real.end(), [](int& input) {
    EXPECT_EQ(1, PredictionThreadPool::s_thread_pool_level);
    std::vector<int> vec = {1, 2, 3, 4};
    PredictionThreadPool::ForEach(vec.begin(), vec.end(), [](int& v) {
      ++v;
      EXPECT_EQ(2, PredictionThreadPool::s_thread_pool_level);
    });
    input = std::accumulate(vec.begin(), vec.end(), input);
  });

  EXPECT_EQ(expect, real);
}

}  // namespace prediction
}  // namespace apollo
