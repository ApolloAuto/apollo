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

#include "modules/perception/lib/config_manager/config_manager.h"

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {

class ConfigManagerTest : public testing::Test {
 protected:
  ConfigManagerTest() : config_manager_(NULL) {}
  virtual ~ConfigManagerTest() {}
  virtual void SetUp() {
    FLAGS_work_root = "modules/perception/data";
    FLAGS_config_manager_path = "./config_manager_test/config_manager.config";
    config_manager_ = ConfigManager::instance();
  }

 protected:
  ConfigManager* config_manager_;
};

TEST_F(ConfigManagerTest, test_Init) {
  EXPECT_TRUE(config_manager_->Init());
  EXPECT_EQ(config_manager_->NumModels(), 3u);
}

TEST_F(ConfigManagerTest, test_Reset) {
  EXPECT_TRUE(config_manager_->Reset());
  std::string wrong_root = "wrong_root";
  config_manager_->SetWorkRoot(wrong_root);
  EXPECT_FALSE(config_manager_->Reset());
  config_manager_->SetWorkRoot(FLAGS_work_root);
}

TEST_F(ConfigManagerTest, test_GetModelConfig) {
  std::string model_name = "ROIFilterTest";
  const ModelConfig* model_config = config_manager_->GetModelConfig(model_name);
  EXPECT_TRUE(model_config != nullptr);
  EXPECT_EQ(model_config->name(), model_name);

  // not exist model.
  model_config = config_manager_->GetModelConfig("noexist");
  EXPECT_EQ(model_config, nullptr);
}

TEST_F(ConfigManagerTest, test_ModelConfig) {
  std::string model_name = "ROIFilterTest";
  ASSERT_TRUE(config_manager_->Init());
  ASSERT_EQ(config_manager_->NumModels(), 3u);
  const ModelConfig* model_config = config_manager_->GetModelConfig(model_name);
  ASSERT_TRUE(model_config != nullptr);
  ASSERT_EQ(model_config->name(), model_name);
  // Check ROIFilterTest param map.

  int int_value = 0;
  EXPECT_TRUE(model_config->GetValue("threshold1", &int_value));
  EXPECT_EQ(int_value, 1);
  EXPECT_TRUE(model_config->GetValue("threshold2", &int_value));
  EXPECT_EQ(int_value, 2);

  std::string str_value;
  EXPECT_TRUE(model_config->GetValue("threshold3", &str_value));
  EXPECT_EQ(str_value, "str3");

  double double_value;
  EXPECT_TRUE(model_config->GetValue("threshold4", &double_value));
  EXPECT_EQ(double_value, 4.0);

  float float_value;
  EXPECT_TRUE(model_config->GetValue("threshold5", &float_value));
  EXPECT_EQ(float_value, 5.0);

  bool bool_value = false;
  EXPECT_TRUE(model_config->GetValue("bool_value_true", &bool_value));
  EXPECT_EQ(bool_value, true);
  EXPECT_TRUE(model_config->GetValue("bool_value_false", &bool_value));
  EXPECT_EQ(bool_value, false);

  std::vector<int> int_list;
  EXPECT_TRUE(model_config->GetValue("array_p1", &int_list));
  EXPECT_EQ(int_list.size(), 3u);
  EXPECT_EQ(int_list[2], 3);

  std::vector<std::string> str_list;
  EXPECT_TRUE(model_config->GetValue("array_p2", &str_list));
  EXPECT_EQ(str_list.size(), 4u);
  EXPECT_EQ(str_list[2], "str3");

  std::vector<double> double_list;
  EXPECT_TRUE(model_config->GetValue("array_p4", &double_list));
  EXPECT_EQ(double_list.size(), 4u);
  EXPECT_EQ(double_list[2], 1.3);

  std::vector<float> float_list;
  EXPECT_TRUE(model_config->GetValue("array_float", &float_list));
  EXPECT_EQ(float_list.size(), 4u);
  EXPECT_FLOAT_EQ(float_list[2], 2.3);

  std::vector<bool> bool_list;
  EXPECT_TRUE(model_config->GetValue("array_bool", &bool_list));
  EXPECT_EQ(bool_list.size(), 4u);
  EXPECT_EQ(bool_list[2], true);

  // not exist
  EXPECT_FALSE(model_config->GetValue("array_p3", &double_list));
  EXPECT_FALSE(model_config->GetValue("array_p3", &int_list));
  EXPECT_FALSE(model_config->GetValue("array_p1", &str_list));
  EXPECT_FALSE(model_config->GetValue("array_p3", &double_value));
  EXPECT_FALSE(model_config->GetValue("array_p3", &int_value));
  EXPECT_FALSE(model_config->GetValue("array_p3", &str_value));
}

}  // namespace perception
}  // namespace apollo
