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

/**
 * @file
 * @brief This file provides several unit tests for the class
 * "TrajectoryConverter".
 */
#include "modules/tools/navi_generator/backend/util/trajectory_converter.h"

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "modules/common/util/util.h"

namespace apollo {
namespace navi_generator {
namespace util {

namespace {
constexpr double kOverlapLenFromSecondBag = 250.0;
}  // namespace

class TrajectoryConverterTest : public testing::Test {
 public:
  virtual void SetUp() {
    converter_ = std::make_unique<TrajectoryConverter>();

    first_namefile_ =
        "modules/tools/navi_generator/backend/util/testdata/"
        "trajectory_converter/1st.xxx";
    second_namefile_ =
        "modules/tools/navi_generator/backend/util/testdata/"
        "trajectory_converter/2nd.xxx";
    third_namefile_ =
        "modules/tools/navi_generator/backend/util/testdata/"
        "trajectory_converter/3rd.xxx";
  }

  bool ProcessBagFile(const std::string& first_bag_filename,
                      const std::string& second_bag_filename,
                      std::vector<std::pair<double, double>>* const waypoints) {
    CHECK_NOTNULL(waypoints);
    if (!converter_->ExtractTrajectoryPointsFromTwoBags(
            first_bag_filename, second_bag_filename,
            kOverlapLenFromSecondBag)) {
      return false;
    }

    if (!converter_->SaveRawTrajectoryPoints()) {
      return false;
    }

    if (!converter_->SmoothTrajectoryPoints()) {
      return false;
    }

    if (!converter_->SaveSmoothedTrajectoryPoints()) {
      return false;
    }

    return true;
  }

 protected:
  std::unique_ptr<TrajectoryConverter> converter_;
  std::string first_namefile_;
  std::string second_namefile_;
  std::string third_namefile_;
  using WayPointPair = std::pair<double, double>;
  std::vector<WayPointPair> waypoints;
};

TEST_F(TrajectoryConverterTest, GoodData1) {
  EXPECT_TRUE(ProcessBagFile(first_namefile_, second_namefile_, &waypoints));
}

TEST_F(TrajectoryConverterTest, GoodData2) {
  EXPECT_TRUE(ProcessBagFile(second_namefile_, third_namefile_, &waypoints));
}

TEST_F(TrajectoryConverterTest, LargeDist) {
  EXPECT_FALSE(ProcessBagFile(first_namefile_, third_namefile_, &waypoints));
}

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
