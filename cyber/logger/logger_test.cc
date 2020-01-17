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

#include "cyber/logger/logger.h"

#include "glog/logging.h"
#include "gtest/gtest.h"

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace logger {

TEST(LoggerTest, WriteAndFlush) {
  Logger logger(google::base::GetLogger(google::INFO));
  time_t timep;
  time(&timep);
  std::string message = "I0909 99:99:99.999999 99999 logger_test.cc:999] ";
  message.append(LEFT_BRACKET);
  message.append("LoggerTest");
  message.append(RIGHT_BRACKET);
  message.append("logger test message\n");
  logger.Write(false, timep, message.c_str(),
               static_cast<int>(message.length()));
  EXPECT_EQ(logger.LogSize(), 0);  // always zero
  logger.Write(true, timep, message.c_str(),
               static_cast<int>(message.length()));
  EXPECT_EQ(logger.LogSize(), 0);  // always zero
  logger.Flush();
}

TEST(LoggerTest, SetLoggerToGlog) {
  google::InitGoogleLogging("LoggerTest2");
  google::SetLogDestination(google::ERROR, "");
  google::SetLogDestination(google::WARNING, "");
  google::SetLogDestination(google::FATAL, "");
  Logger* logger = new Logger(google::base::GetLogger(google::INFO));
  google::base::SetLogger(FLAGS_minloglevel, logger);

  ALOG_MODULE("LoggerTest2", INFO) << "test set logger to glog";
  ALOG_MODULE("LoggerTest2", WARN) << "test set logger to glog";
  ALOG_MODULE("LoggerTest2", ERROR) << "test set logger to glog";
  logger = nullptr;
  google::ShutdownGoogleLogging();
}

}  // namespace logger
}  // namespace cyber
}  // namespace apollo
