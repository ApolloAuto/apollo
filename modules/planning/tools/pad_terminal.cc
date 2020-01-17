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

#include "modules/planning/proto/pad_msg.pb.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"

namespace {

using apollo::cyber::CreateNode;
using apollo::cyber::Node;
using apollo::cyber::Writer;
using apollo::planning::DrivingAction;
using apollo::planning::PadMessage;

class PadTerminal {
 public:
  PadTerminal() : node_(CreateNode("planning_pad_terminal")) {}
  void init() {
    pad_writer_ = node_->CreateWriter<PadMessage>(FLAGS_planning_pad_topic);
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
    pad.set_action(DrivingAction(action));
    if (action == DrivingAction::FOLLOW) {
      AINFO << "sending FOLLOW action command.";
    } else if (action == DrivingAction::CHANGE_LEFT) {
      AINFO << "sending CHANGE LEFT action command.";
    } else if (action == DrivingAction::CHANGE_RIGHT) {
      AINFO << "sending CHANGE RIGHT action command.";
    } else if (action == DrivingAction::PULL_OVER) {
      AINFO << "sending PULL OVER action command.";
    } else if (action == DrivingAction::STOP) {
      AINFO << "sending STOP action command.";
    } else if (action == DrivingAction::RESUME_CRUISE) {
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
        case 5:
          send(DrivingAction::RESUME_CRUISE);
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
