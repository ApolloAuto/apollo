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

#include "modules/dreamview/backend/data_collection_monitor/data_collection_monitor.h"

#include "gtest/gtest.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/json_util.h"

#include "modules/dreamview/backend/common/dreamview_gflags.h"

using apollo::canbus::Chassis;

namespace apollo {
namespace dreamview {

using apollo::common::util::JsonUtil;

class DataCollectionMonitorTest : public ::testing::Test {
 public:
  static void SetUpTestCase() {
    cyber::GlobalData::Instance()->EnableSimulationMode();

    std::unique_ptr<cyber::Node> node =
        cyber::CreateNode("data_collection_monitor_test");
  }

  virtual void SetUp() {
    data_collection_monitor_.reset(new DataCollectionMonitor());
    data_collection_monitor_->Start();
  }

 protected:
  DataCollectionMonitorTest() {
    FLAGS_default_data_collection_config_path =
        "/apollo/modules/dreamview/backend/testdata/"
        "data_collection_table_test.pb.txt";
  }

  std::unique_ptr<DataCollectionMonitor> data_collection_monitor_;
};

TEST_F(DataCollectionMonitorTest, ConstructCategories) {
  data_collection_monitor_->Start();
  const auto& scenario =
      data_collection_monitor_->categories_.find("Category Construction Test");
  EXPECT_NE(scenario, data_collection_monitor_->categories_.end());

  const auto& categories = scenario->second;
  EXPECT_EQ(6, categories.size());
  EXPECT_EQ("mps < 10, Throttle == 30%", categories[0].name());
  EXPECT_EQ("mps < 10, Throttle != 30%", categories[1].name());
  EXPECT_EQ("mps < 10, Throttle deadzone ~ 35%", categories[2].name());
  EXPECT_EQ("mps >= 10, Throttle == 30%", categories[3].name());
  EXPECT_EQ("mps >= 10, Throttle != 30%", categories[4].name());
  EXPECT_EQ("mps >= 10, Throttle deadzone ~ 35%", categories[5].name());

  const auto& criterion = categories[0].criterion();
  EXPECT_EQ("gear_location", criterion[0].field());
  EXPECT_EQ(ComparisonOperator::EQUAL, criterion[0].comparison_operator());
  EXPECT_EQ(1, criterion[0].value());

  EXPECT_EQ("speed_mps", criterion[1].field());
  EXPECT_EQ(ComparisonOperator::LESS_THAN, criterion[1].comparison_operator());
  EXPECT_EQ(10, criterion[1].value());

  EXPECT_EQ("throttle_percentage", criterion[2].field());
  EXPECT_EQ(ComparisonOperator::EQUAL, criterion[2].comparison_operator());
  EXPECT_EQ(30, criterion[2].value());
}

TEST_F(DataCollectionMonitorTest, UpdateCollectionProgress) {
  data_collection_monitor_->Start();

  auto chassis = std::make_shared<Chassis>();
  chassis->set_speed_mps(10.0f);
  chassis->set_throttle_percentage(31.1f);
  chassis->set_brake_percentage(30.0f);
  chassis->set_steering_percentage(-30.0f);

  std::string scenarioName = "Comparison Operator Test";
  {
    data_collection_monitor_->OnChassis(chassis);
    nlohmann::json progress = data_collection_monitor_->GetProgressAsJson();

    auto scenario = progress.find(scenarioName);
    EXPECT_NE(scenario, progress.end());

    double value;
    bool hasField;

    hasField = JsonUtil::GetNumberFromJson(*scenario, "mps < 10", &value);
    EXPECT_TRUE(hasField);
    EXPECT_DOUBLE_EQ(0.0, value);

    hasField = JsonUtil::GetNumberFromJson(*scenario, "mps >= 10", &value);
    EXPECT_TRUE(hasField);
    EXPECT_DOUBLE_EQ(0.0, value);

    hasField =
        JsonUtil::GetNumberFromJson(*scenario, "Throttle == 30%", &value);
    EXPECT_TRUE(hasField);
    EXPECT_DOUBLE_EQ(0.0, value);

    hasField =
        JsonUtil::GetNumberFromJson(*scenario, "Throttle != 30%", &value);
    EXPECT_TRUE(hasField);
    EXPECT_DOUBLE_EQ(0.0, value);

    hasField = JsonUtil::GetNumberFromJson(*scenario, "Brake <= 30%", &value);
    EXPECT_TRUE(hasField);
    EXPECT_DOUBLE_EQ(0.0, value);

    hasField = JsonUtil::GetNumberFromJson(*scenario, "Brake > 30%", &value);
    EXPECT_TRUE(hasField);
    EXPECT_DOUBLE_EQ(0.0, value);

    hasField =
        JsonUtil::GetNumberFromJson(*scenario, "Left steering < 20%", &value);
    EXPECT_TRUE(hasField);
    EXPECT_DOUBLE_EQ(0.0, value);

    hasField = JsonUtil::GetNumberFromJson(*scenario,
                                           "Right steering 20% ~ 40%", &value);
    EXPECT_TRUE(hasField);
    EXPECT_DOUBLE_EQ(0.0, value);

    hasField = JsonUtil::GetNumberFromJson(*scenario, "Throttle deadzone ~ 35%",
                                           &value);
    EXPECT_TRUE(hasField);
    EXPECT_DOUBLE_EQ(0.0, value);
  }

  {
    data_collection_monitor_->OnChassis(chassis);
    data_collection_monitor_->OnChassis(chassis);
    nlohmann::json progress = data_collection_monitor_->GetProgressAsJson();

    auto scenario = progress.find(scenarioName);
    EXPECT_NE(scenario, progress.end());

    float value;
    bool hasField;

    hasField = JsonUtil::GetNumberFromJson(*scenario, "Brake > 30%", &value);
    EXPECT_TRUE(hasField);
    EXPECT_DOUBLE_EQ(0.0, value);

    hasField = JsonUtil::GetNumberFromJson(*scenario, "Brake <= 30%", &value);
    EXPECT_TRUE(hasField);
    EXPECT_DOUBLE_EQ(75.0, value);

    hasField =
        JsonUtil::GetNumberFromJson(*scenario, "Throttle != 30%", &value);
    EXPECT_TRUE(hasField);
    EXPECT_DOUBLE_EQ(75.0, value);

    hasField = JsonUtil::GetNumberFromJson(*scenario,
                                           "Right steering 20% ~ 40%", &value);
    EXPECT_TRUE(hasField);
    EXPECT_DOUBLE_EQ(75.0, value);

    hasField = JsonUtil::GetNumberFromJson(*scenario, "Throttle deadzone ~ 35%",
                                           &value);
    EXPECT_TRUE(hasField);
    EXPECT_DOUBLE_EQ(75.0, value);
  }

  data_collection_monitor_->Stop();
  EXPECT_FALSE(data_collection_monitor_->IsEnabled());
}

}  // namespace dreamview
}  // namespace apollo
