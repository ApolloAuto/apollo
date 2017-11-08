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

TEST(HMITest, RunComponentCommand) {
  google::protobuf::Map<std::string, Component> components;
  // Fail on not-exist component.
  EXPECT_NE(0, HMI::RunComponentCommand(components, "Module", "NotExist"));

  auto *commands =
      components.insert({"A", {}}).first->second.mutable_supported_commands();
  {
    // Succeed on single command.
    commands->insert({"SingleCommand", "ls"});
    EXPECT_EQ(0, HMI::RunComponentCommand(components, "A", "SingleCommand"));
  }
  {
    // Succeed on complex command.
    commands->insert({"ComplexCommand", "ls /dev/null"});
    EXPECT_EQ(0, HMI::RunComponentCommand(components, "A", "ComplexCommand"));
  }
  {
    // Fail on bad command.
    commands->insert({"BadCommand", "ls /dev/null/not_exist"});
    EXPECT_NE(0, HMI::RunComponentCommand(components, "A", "BadCommand"));
  }
}

}  // namespace dreamview
}  // namespace apollo
