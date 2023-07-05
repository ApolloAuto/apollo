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
#include "modules/perception/lib/config_manager/config_manager.h"

#include "gtest/gtest.h"

#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {
namespace lib {

class ConfigManagerTest : public testing::Test {
 protected:
  ConfigManagerTest() : config_manager_(NULL) {}
  virtual ~ConfigManagerTest() {}
  virtual void SetUp() {
    char cyber_path[80] = "CYBER_PATH=";
    putenv(cyber_path);
    char module_path[80] = "MODULE_PATH=";
    putenv(module_path);
    FLAGS_config_manager_path = "/apollo/modules/perception/testdata/lib/conf";
    config_manager_ = ConfigManager::Instance();
    ASSERT_TRUE(config_manager_ != nullptr);
  }

 protected:
  ConfigManager* config_manager_;
};

TEST_F(ConfigManagerTest, TestInit) {
  config_manager_->inited_ = true;
  EXPECT_TRUE(config_manager_->Init());
  EXPECT_TRUE(config_manager_->Reset());
  config_manager_->set_work_root("");
  EXPECT_TRUE(config_manager_->Init());
  EXPECT_TRUE(config_manager_->Reset());
  EXPECT_EQ(config_manager_->NumModels(), 2u);
  ConfigManager config_manager;
  config_manager.set_work_root("");
  EXPECT_EQ(config_manager.work_root(), "");
}

TEST_F(ConfigManagerTest, TestGetModelConfig) {
  std::string model_name = "FrameClassifier";
  const ModelConfig* model_config = nullptr;

  EXPECT_TRUE(config_manager_->GetModelConfig(model_name, &model_config));
  ASSERT_TRUE(model_config != nullptr);
  EXPECT_EQ(model_config->name(), model_name);

  // not exist model.
  model_config = nullptr;
  EXPECT_FALSE(config_manager_->GetModelConfig("noexist", &model_config));
  EXPECT_EQ(model_config, nullptr);
}

TEST_F(ConfigManagerTest, TestModelConfig) {
  std::string model_name = "FrameClassifier";
  const ModelConfig* model_config = nullptr;
  ASSERT_TRUE(config_manager_->Init());
  ASSERT_EQ(config_manager_->NumModels(), 2u);
  ASSERT_FALSE(
      config_manager_->GetModelConfig("FrameClassifier1", &model_config));
  ASSERT_TRUE(config_manager_->GetModelConfig(model_name, &model_config));
  ASSERT_EQ(model_config->name(), model_name);

  // Check FrameClassifier param map.
  int int_value = 0;
  EXPECT_TRUE(model_config->get_value("threshold1", &int_value));
  EXPECT_EQ(int_value, 1);
  EXPECT_TRUE(model_config->get_value("threshold2", &int_value));
  EXPECT_EQ(int_value, 2);

  std::string str_value;
  EXPECT_TRUE(model_config->get_value("threshold3", &str_value));
  EXPECT_EQ(str_value, "str3");

  double double_value;
  EXPECT_TRUE(model_config->get_value("threshold4", &double_value));
  EXPECT_EQ(double_value, 4.0);

  float float_value;
  EXPECT_TRUE(model_config->get_value("threshold5", &float_value));
  EXPECT_EQ(float_value, 5.0);

  bool bool_value = false;
  EXPECT_TRUE(model_config->get_value("bool_value_true", &bool_value));
  EXPECT_TRUE(bool_value);
  EXPECT_TRUE(model_config->get_value("bool_value_false", &bool_value));
  EXPECT_FALSE(bool_value);

  std::vector<int> int_list;
  EXPECT_TRUE(model_config->get_value("array_p1", &int_list));
  EXPECT_EQ(int_list.size(), 3u);
  EXPECT_EQ(int_list[2], 3);

  std::vector<std::string> str_list;
  EXPECT_TRUE(model_config->get_value("array_p2", &str_list));
  EXPECT_EQ(str_list.size(), 4u);
  EXPECT_EQ(str_list[2], "str3");

  std::vector<double> double_list;
  EXPECT_TRUE(model_config->get_value("array_p4", &double_list));
  EXPECT_EQ(double_list.size(), 4u);
  EXPECT_EQ(double_list[2], 1.3);

  std::vector<bool> bool_list;
  EXPECT_TRUE(model_config->get_value("array_bool", &bool_list));
  EXPECT_EQ(bool_list.size(), 4u);
  EXPECT_TRUE(bool_list[2]);

  // not exist
  EXPECT_FALSE(model_config->get_value("array_p3", &double_list));
  EXPECT_FALSE(model_config->get_value("array_p3", &int_list));
  EXPECT_FALSE(model_config->get_value("array_p1", &str_list));
  EXPECT_FALSE(model_config->get_value("array_p3", &double_value));
  EXPECT_FALSE(model_config->get_value("array_p3", &int_value));
  EXPECT_FALSE(model_config->get_value("array_p3", &str_value));
}

TEST_F(ConfigManagerTest, TestConfigManagerError) {
  ConfigManagerError error("config manager error");
  EXPECT_EQ(error.What(), "config manager error");
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo
