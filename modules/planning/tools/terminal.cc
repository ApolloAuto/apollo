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

#include <iostream>
#include <thread>

#include "ros/include/ros/ros.h"

#include "modules/planning/proto/pad_msg.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"

namespace {

using apollo::common::adapter::AdapterManager;
using apollo::planning::DrivingAction;
using apollo::planning::PadMessage;

void help() {
  AINFO << "COMMAND:0~10\n";
  AINFO << "\t0: follow.";
  AINFO << "\t1: change left.";
  AINFO << "\t2: change right.";
  AINFO << "\t3: pull over.";
  AINFO << "\t4: stop.";
  AINFO << "\t10: exit.";
  AINFO << "\tother number: print help.";
}

void send(int action) {
  PadMessage pad;
  pad.set_action(DrivingAction(action));
  if (action == DrivingAction::FOLLOW)
    AINFO << "sending follow action command.";
  else if (action == DrivingAction::CHANGE_LEFT)
    AINFO << "sending change left action command.";
  else if (action == DrivingAction::CHANGE_RIGHT)
    AINFO << "sending change right action command.";
  else if (action == DrivingAction::PULL_OVER)
    AINFO << "sending pull over action command.";
  else if (action == DrivingAction::STOP)
    AINFO << "sending stop action command.";
  AdapterManager::FillPlanningPadHeader("planning_terminal", &pad);
  AdapterManager::PublishPlanningPad(pad);
  AINFO << "send pad_message OK";
}

void terminal_thread_func() {
  int mode = 0;
  bool should_exit = false;
  while (std::cin >> mode) {
    switch (mode) {
      case 0:
        send(DrivingAction::FOLLOW);
        break;
      case 1:
        send(DrivingAction::CHANGE_LEFT);
        break;
      case 2:
        send(DrivingAction::CHANGE_RIGHT);
        break;
      case 3:
        send(DrivingAction::PULL_OVER);
        break;
      case 4:
        send(DrivingAction::STOP);
        break;
      case 10:
        should_exit = true;
        break;
      default:
        help();
        break;
    }
    if (should_exit) {
      ros::shutdown();
      break;
    }
  }
}

}  // namespace

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_v = 3;

  google::ParseCommandLineFlags(&argc, &argv, true);

  using apollo::common::adapter::AdapterConfig;
  using apollo::common::adapter::AdapterManager;
  using apollo::common::adapter::AdapterManagerConfig;

  ros::init(argc, argv, "planning_terminal");

  AdapterManagerConfig config;
  config.set_is_ros(true);
  {
    auto *sub_config = config.add_config();
    sub_config->set_mode(AdapterConfig::PUBLISH_ONLY);
    sub_config->set_type(AdapterConfig::PLANNING_PAD);
  }

  AdapterManager::Init(config);

  help();
  std::thread terminal_thread(terminal_thread_func);
  ros::spin();
  terminal_thread.join();
  return 0;
}
