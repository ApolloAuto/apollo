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

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <string>

#include "cybertron/logger/logger.h"
#include "cybertron/time/time.h"

namespace apollo {
namespace cybertron {
namespace logger {

TEST(LoggerTest, init_and_write) {
  Logger logger(google::base::GetLogger(google::INFO));
  time_t timep;
  time(&timep);
  std::string message = "cybertron logger test";
  logger.Write(false, timep, message.c_str(), 20);
  message = "**[CybertronLoggerTest]** cybertron logger test";
  logger.Write(false, timep, message.c_str(), 20);
  logger.LogSize();
  logger.Flush();
}

}  // namespace logger
}  // namespace cybertron
}  // namespace apollo
