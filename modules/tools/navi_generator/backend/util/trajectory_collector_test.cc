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
#include "modules/tools/navi_generator/backend/util/trajectory_collector.h"

#include <ros/ros.h>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "modules/common/log.h"

namespace apollo {
namespace navi_generator {
namespace util {

class TrajectoryCollectorTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    collect_ = std::unique_ptr<TrajectoryCollector>(new TrajectoryCollector());
  }

 protected:
  std::unique_ptr<TrajectoryCollector> collect_;
  CollectorOptions options_;
};

TEST_F(TrajectoryCollectorTest, Init) {
  // Config options
  std::vector<std::string> topics = {"/apollo/sensor/gnss/best_pose",
                                     "/apollo/localization/pose"};
  options_.topics = topics;
  // EXPECT_TRUE(collect_->Init(options_));
}

TEST_F(TrajectoryCollectorTest, Start) {
  // options_.split = true;
  // EXPECT_TRUE(collect_->Init(options_));
  // EXPECT_TRUE(collect_->Start());
}

TEST_F(TrajectoryCollectorTest, Stop) {}
TEST_F(TrajectoryCollectorTest, UpdateOptions) {}
TEST_F(TrajectoryCollectorTest, CheckDuration) {}

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
