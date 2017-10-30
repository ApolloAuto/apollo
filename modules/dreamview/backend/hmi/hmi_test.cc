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

#include "modules/dreamview/backend/hmi/hmi.h"

#include "gtest/gtest.h"

namespace apollo {
namespace dreamview {

using google::protobuf::Map;

TEST(HMITest, UpdateHMIStatus) {
  HMI hmi(nullptr);

  EXPECT_GT(hmi.config_.modules_size(), 0);
  EXPECT_EQ(hmi.status_.modules_size(), hmi.config_.modules_size());

  // The initial status should be UNINITIALIZED.
  const auto &module_status_iter = hmi.status_.modules().begin();
  EXPECT_EQ(ModuleStatus::UNINITIALIZED, module_status_iter->second.status());

  // Set it to STARTED.
  ModuleStatus new_module_status;
  new_module_status.set_status(ModuleStatus::STARTED);
  HMIStatus new_status;
  new_status.mutable_modules()->insert({module_status_iter->first,
                                        new_module_status});
  hmi.OnHMIStatus(new_status);

  EXPECT_EQ(ModuleStatus::STARTED, module_status_iter->second.status());
}

TEST(HMITest, ExecuteComponentCommand) {
  Map<std::string, Component> modules;
  // Fail on not-exist component.
  EXPECT_NE(0, HMI::ExecuteComponentCommand(modules, "Module", "Any"));

  auto *module_commands = modules.insert({"Module", {}}).first
      ->second.mutable_supported_commands();
  {
    // Succeed on single command.
    module_commands->insert({"SingleCommand", "ls"});
    EXPECT_EQ(0, HMI::ExecuteComponentCommand(modules, "Module",
                                              "SingleCommand"));
  }
  {
    // Succeed on complex command.
    module_commands->insert({"ComplexCommand", "ls /dev/null"});
    EXPECT_EQ(0, HMI::ExecuteComponentCommand(modules, "Module",
                                              "ComplexCommand"));
  }
  {
    // Fail on bad command.
    module_commands->insert({"BadCommand", "ls /dev/null/not_exist"});
    EXPECT_NE(0, HMI::ExecuteComponentCommand(modules, "Module", "BadCommand"));
  }
}

}  // namespace dreamview
}  // namespace apollo
