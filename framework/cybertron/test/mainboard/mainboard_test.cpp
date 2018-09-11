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

#include "gtest/gtest.h"

#include "cybertron/common/environment.h"
#include "cybertron/init.h"
#include "cybertron/mainboard/module_argument.h"
#include "cybertron/mainboard/module_controller.h"

namespace apollo {
namespace cybertron {

TEST(ModuleControllerTest, parsearg) {
  apollo::cybertron::mainboard::ModuleArgument argment;
  std::string dag_path =
      common::GetEnv("CYBERTRON_PATH") + "/dag/perception.dag";
  char* argv[3] = {"mainboard", "-d", ""};
  argv[2] = const_cast<char*>(dag_path.c_str());

  EXPECT_TRUE(argment.ParseArgument(3, argv));
  EXPECT_TRUE("mainboard" == argment.GetBinaryName());
  EXPECT_TRUE("mainboard_default" == argment.GetProcessName());

  apollo::cybertron::mainboard::ModuleController module_ctrl(argment);
  EXPECT_TRUE(1 == argment.GetDAGConfList().size());
  module_ctrl.LoadAll();
  module_ctrl.Clear();
}

}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  apollo::cybertron::Init(argv[0]);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
