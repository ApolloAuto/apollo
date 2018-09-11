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

#include "cybertron/common/types.h"
#include "cybertron/common/environment.h"
#include "cybertron/common/file.h"
#include "cybertron/common/log.h"
#include "cybertron/init.h"
#include "cybertron/mainboard/module_argument.h"
#include "cybertron/mainboard/module_controller.h"
#include "gflags/gflags.h"

using apollo::cybertron::common::WorkRoot;
using apollo::cybertron::common::GetAbsolutePath;
using apollo::cybertron::common::GetProtoFromFile;
using apollo::cybertron::common::EnsureDirectory;
using apollo::cybertron::mainboard::ModuleArgument;
using apollo::cybertron::mainboard::ModuleController;

int main(int argc, char** argv) {
  // Initialize cybertron internal static objects
  apollo::cybertron::Init(argv[0]);
  google::SetUsageMessage("This program used for load dag and run user apps.");

  // parser the argument
  ModuleArgument module_args;
  if (!module_args.ParseArgument(argc, argv)) {
    AERROR << "parse argument error!";
    module_args.DisplayUsage();
    return -1;
  }

  // start module
  ModuleController controller(module_args);
  if (!controller.Init()) {
    AERROR << "module start error.";
    return -1;
  }

  while (!apollo::cybertron::IsShutdown()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  controller.Clear();
  AINFO << "Exit mainboard.";
  return 0;
}
