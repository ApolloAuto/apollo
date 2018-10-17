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

#include "cybertron/common/log.h"
#include "cybertron/init.h"
#include "cybertron/mainboard/module_argument.h"
#include "cybertron/mainboard/module_controller.h"
#include "cybertron/state.h"

#include "gflags/gflags.h"

using apollo::cybertron::mainboard::ModuleArgument;
using apollo::cybertron::mainboard::ModuleController;

int main(int argc, char** argv) {
  google::SetUsageMessage("we use this program to load dag and run user apps.");

  // initialize cybertron
  apollo::cybertron::Init(argv[0]);

  // parse the argument
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

  apollo::cybertron::WaitForShutdown();
  controller.Clear();
  AINFO << "exit mainboard.";

  return 0;
}
