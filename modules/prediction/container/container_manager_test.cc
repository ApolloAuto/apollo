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

#include <string>

#include "gtest/gtest.h"

#include "modules/common/util/file.h"
#include "modules/prediction/container/container_manager.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;

class ContainerManagerTest : public ::testing::Test {
 public:
  virtual void SetUp() { manager_ = ContainerManager::instance(); }

 protected:
  ContainerManager *manager_ = nullptr;
  common::adapter::AdapterManagerConfig conf_;
};

TEST_F(ContainerManagerTest, GetContainer) {
  std::string conf_file = "modules/prediction/testdata/adapter_conf.pb.txt";
  bool ret_load_conf = common::util::GetProtoFromFile(conf_file, &conf_);
  EXPECT_TRUE(ret_load_conf);
  EXPECT_TRUE(conf_.IsInitialized());

  manager_->Init(conf_);
  EXPECT_TRUE(manager_->GetContainer(AdapterConfig::PERCEPTION_OBSTACLES) !=
              nullptr);
  EXPECT_TRUE(manager_->GetContainer(AdapterConfig::LOCALIZATION) != nullptr);
  EXPECT_TRUE(manager_->GetContainer(AdapterConfig::CONTROL_COMMAND) ==
              nullptr);
}

}  // namespace prediction
}  // namespace apollo
