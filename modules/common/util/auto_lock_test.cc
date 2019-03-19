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

#include "modules/common/util/auto_lock.h"
#include <thread>
#include "gtest/gtest.h"

#define MAX_COUNT       10000000
#define EXPECTED_RESULT 20000000

std::mutex g_mutex;
int g_int = 0;

void function() {
  for (int i = 0; i < MAX_COUNT; i++) {
    AutoLock<std::mutex> autolock(&g_mutex);
    g_int++;
  }
}

TEST(AutoLockTest, test_autolock) {
  std::thread t1(function);
  std::thread t2(function);

  t1.join();
  t2.join();

  AutoLock<std::mutex> autolock(&g_mutex);
  EXPECT_EQ(g_int, EXPECTED_RESULT);
  autolock.UnLock();
}
