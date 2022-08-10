/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/time/time.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/common_msgs/planning_msgs/pad_msg.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

namespace {

using apollo::cyber::CreateNode;
using apollo::cyber::Node;
using apollo::cyber::Writer;
using apollo::cyber::common::GetProtoFromFile;
using apollo::planning::PadMessage;
using apollo::planning::PlanningConfig;

class PadTerminal {
 public:
  PadTerminal() : node_(CreateNode("planning_pad_terminal")) {}
  void init() {
    const std::string planning_config_file =
        "/apollo/modules/planning/conf/planning_config.pb.txt";
    PlanningConfig planning_config;
    ACHECK(GetProtoFromFile(planning_config_file, &planning_config))
        << "failed to load planning config file " << planning_config_file;

    pad_writer_ = node_->CreateWriter<PadMessage>(
        planning_config.topic_config().planning_pad_topic());
    terminal_thread_.reset(new std::thread([this] { terminal_thread_func(); }));
  }
  void help() {
    AINFO << "COMMAND:0~10\n";
    AINFO << "\t0: follow";
    AINFO << "\t1: change left";
    AINFO << "\t2: change right";
    AINFO << "\t3: pull over";
    AINFO << "\t4: stop";
    AINFO << "\t5: resume cruise";
    AINFO << "\t10: exit";
    AINFO << "\tother number: print help";
  }

  void send(int action) {
    PadMessage pad;
    pad.set_action(PadMessage::DrivingAction(action));
    if (action == PadMessage::FOLLOW) {
      AINFO << "sending FOLLOW action command.";
    } else if (action == PadMessage::CHANGE_LEFT) {
      AINFO << "sending CHANGE LEFT action command.";
    } else if (action == PadMessage::CHANGE_RIGHT) {
      AINFO << "sending CHANGE RIGHT action command.";
    } else if (action == PadMessage::PULL_OVER) {
      AINFO << "sending PULL OVER action command.";
    } else if (action == PadMessage::STOP) {
      AINFO << "sending STOP action command.";
    } else if (action == PadMessage::RESUME_CRUISE) {
      AINFO << "sending RESUME CRUISE action command.";
    }
    apollo::common::util::FillHeader("terminal", &pad);
    pad_writer_->Write(pad);
    AINFO << "send pad_message OK";
  }

  void terminal_thread_func() {
    int mode = 0;
    bool should_exit = false;
    while (std::cin >> mode) {
      switch (mode) {
        case 0:
          send(PadMessage::FOLLOW);
          break;
        case 1:
          send(PadMessage::CHANGE_LEFT);
          break;
        case 2:
          send(PadMessage::CHANGE_RIGHT);
          break;
        case 3:
          send(PadMessage::PULL_OVER);
          break;
        case 4:
          send(PadMessage::STOP);
          break;
        case 5:
          send(PadMessage::RESUME_CRUISE);
          break;
        case 10:
          should_exit = true;
          break;
        default:
          help();
          break;
      }
      if (should_exit) {
        break;
      }
    }
  }
  void stop() { terminal_thread_->join(); }

 private:
  std::unique_ptr<std::thread> terminal_thread_;
  std::shared_ptr<Writer<PadMessage>> pad_writer_;
  std::shared_ptr<Node> node_;
};

}  // namespace

int main(int argc, char **argv) {
  apollo::cyber::Init("planning_pad_terminal");
  FLAGS_alsologtostderr = true;
  FLAGS_v = 3;
  google::ParseCommandLineFlags(&argc, &argv, true);
  PadTerminal pad_terminal;
  pad_terminal.init();
  pad_terminal.help();
  apollo::cyber::WaitForShutdown();
  pad_terminal.stop();
  return 0;
}
