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

#include "cyber/logger/async_logger.h"
#include "cyber/time/time.h"

namespace apollo {
namespace cyber {
namespace logger {

const char fatalMsg[] = "Fatal [AsyncLoggerTest] Test AsyncLogger String\n";
const char errorMsg[] = "Error [AsyncLoggerTest] Test AsyncLogger String\n";
const char warningMsg[] = "Warning [AsyncLoggerTest] Test AsyncLogger String\n";
const char inforMsg[] = "Info [AsyncLoggerTest] Test AsyncLogger String\n";
const uint64_t timestamp = 1546272000LL;

TEST(AsyncLoggerTest, WriteAndFlush) {
  // Start async logger
  google::InitGoogleLogging("AsyncLoggerTest");
  AsyncLogger* a = new AsyncLogger(google::base::GetLogger(google::INFO));
  google::base::SetLogger(FLAGS_minloglevel, a);
  a->Start();

  a->Write(true, 0, "", 0);

  a->Flush();
  EXPECT_EQ(a->LogSize(), 0);

  a->Write(false, timestamp, fatalMsg, strlen(fatalMsg) - 1);
  EXPECT_EQ(a->LogSize(), 0);

  a->Write(false, timestamp, errorMsg, strlen(errorMsg) - 1);
  EXPECT_EQ(a->LogSize(), 0);

  a->Write(false, timestamp, warningMsg, strlen(warningMsg) - 1);
  EXPECT_EQ(a->LogSize(), 0);

  a->Write(false, timestamp, inforMsg, strlen(inforMsg) - 1);
  EXPECT_EQ(a->LogSize(), 0);

  a->Flush();
  EXPECT_EQ(a->LogSize(), 0);

  // Stop async logger
  a->Stop();
  a = nullptr;
  google::ShutdownGoogleLogging();
}

TEST(AsyncLoggerTest, WriteAndFlushWithDefaultSize) {
  // Start async logger
  google::InitGoogleLogging("AsyncLoggerTest");
  AsyncLogger* a = new AsyncLogger(google::base::GetLogger(google::INFO));
  google::base::SetLogger(FLAGS_minloglevel, a);
  a->Start();

  a->Write(false, timestamp, fatalMsg, strlen(fatalMsg) - 1);
  EXPECT_EQ(a->LogSize(), 0);

  a->Write(false, timestamp, errorMsg, strlen(errorMsg) - 1);
  EXPECT_EQ(a->LogSize(), 0);

  a->Write(false, timestamp, warningMsg, strlen(warningMsg) - 1);
  EXPECT_EQ(a->LogSize(), 0);

  a->Write(false, timestamp, inforMsg, strlen(inforMsg) - 1);
  EXPECT_EQ(a->LogSize(), 0);

  a->Flush();
  EXPECT_EQ(a->LogSize(), 0);

  // Stop async logger
  a->Stop();
  a = nullptr;
  google::ShutdownGoogleLogging();
}

TEST(AsyncLoggerTest, StartAndStop) {
  // Start async logger
  google::InitGoogleLogging("AsyncLoggerTest");

  AsyncLogger* a = new AsyncLogger(google::base::GetLogger(google::INFO));
  google::base::SetLogger(FLAGS_minloglevel, a);
  a->Start();
  a->LogThread()->detach();

  // Stop async logger
  a->Stop();
  a = nullptr;
  google::ShutdownGoogleLogging();
}

}  // namespace logger
}  // namespace cyber
}  // namespace apollo
