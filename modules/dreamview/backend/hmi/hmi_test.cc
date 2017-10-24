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

using google::protobuf::RepeatedPtrField;

TEST(HMITest, UpdateHMIStatus) {
  HMI hmi(nullptr);

  EXPECT_GT(hmi.config_.modules_size(), 0);
  EXPECT_EQ(hmi.status_.modules_size(), hmi.config_.modules_size());

  // The initial status should be UNINITIALIZED.
  const auto &module_name = hmi.config_.modules(0).name();
  EXPECT_EQ(ModuleStatus::UNINITIALIZED,
            hmi.GetModuleStatus(module_name)->status());

  // Set it to STARTED.
  HMIStatus new_status;
  auto *module_status = new_status.add_modules();
  module_status->set_name(module_name);
  module_status->set_status(ModuleStatus::STARTED);
  hmi.OnHMIStatus(new_status);

  EXPECT_EQ(ModuleStatus::STARTED, hmi.GetModuleStatus(module_name)->status());
}

TEST(HMITest, ExecuteComponentCommand) {
  RepeatedPtrField<Component> modules;
  // Fail.
  EXPECT_NE(0, HMI::ExecuteComponentCommand(modules, "Module", "Any"));

  auto *module = modules.Add();
  module->set_name("Module");
  {
    auto *command = module->add_supported_commands();
    command->set_name("SingleCommand");
    *command->add_command() = "echo OK";
    // Succeed.
    EXPECT_EQ(0, HMI::ExecuteComponentCommand(modules, "Module",
                                              "SingleCommand"));
  }
  {
    auto *command = module->add_supported_commands();
    command->set_name("ComplexCommand");
    *command->add_command() = "ls";
    *command->add_command() = "/dev/null";
    // Succeed.
    EXPECT_EQ(0, HMI::ExecuteComponentCommand(modules, "Module",
                                              "ComplexCommand"));
  }
  {
    auto *command = module->add_supported_commands();
    command->set_name("BadCommand");
    *command->add_command() = "ls /dev/null/not_exist";
    // Fail.
    EXPECT_NE(0, HMI::ExecuteComponentCommand(modules, "Module", "BadCommand"));
  }
}

}  // namespace dreamview
}  // namespace apollo
