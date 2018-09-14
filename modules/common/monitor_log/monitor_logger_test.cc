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
#include "modules/common/monitor_log/monitor_logger.h"

#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace monitor {

class MonitorTest : public MonitorLogger {
 public:
  explicit MonitorTest(const MonitorMessageItem::MessageSource &source)
      : MonitorLogger(source) {}

 private:
  void DoPublish(MonitorMessage *) const override {}
};

TEST(MonitorTest, Publish) {
  MonitorTest monitor(MonitorMessageItem::CONTROL);
  //  std::vector<std::pair<MonitorMessageItem::LogLevel, std::string>> items{
  //      {MonitorMessageItem::INFO, "info message"},
  //      {MonitorMessageItem::WARN, "warn message"},
  //      {MonitorMessageItem::ERROR, "error message"},
  //  };
  //  monitor.Publish(items);
}

}  // namespace monitor
}  // namespace common
}  // namespace apollo
