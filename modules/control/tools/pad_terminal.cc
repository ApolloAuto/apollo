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

#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/time/time.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/control/proto/pad_msg.pb.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/time/time.h"
#include "modules/common/util/message_util.h"
#include "modules/control/common/control_gflags.h"

namespace {

using apollo::canbus::Chassis;
using apollo::common::time::Clock;
using apollo::control::DrivingAction;
using apollo::control::PadMessage;
using apollo::cyber::CreateNode;
using apollo::cyber::Node;
using apollo::cyber::Reader;
using apollo::cyber::Writer;

class PadTerminal {
 public:
  PadTerminal() : node_(CreateNode("pad_terminal")) {}
  void init() {
    chassis_reader_ = node_->CreateReader<Chassis>(
        FLAGS_chassis_topic, [this](const std::shared_ptr<Chassis> &chassis) {
          on_chassis(*chassis);
        });

    pad_writer_ = node_->CreateWriter<PadMessage>(FLAGS_pad_topic);
    terminal_thread_.reset(new std::thread([this] { terminal_thread_func(); }));
  }
  void help() {
    AINFO << "COMMAND:\n";
    AINFO << "\t0: reset to manual drive mode.";
    AINFO << "\t1: auto drive mode.";
    AINFO << "\tctrl + c: exit.";
    AINFO << "\tother: print help.";
  }
  void send(int cmd_type) {
    PadMessage pad;
    if (cmd_type == RESET_COMMAND) {
      pad.set_action(DrivingAction::RESET);
      AINFO << "sending reset action command.";
    } else if (cmd_type == AUTO_DRIVE_COMMAND) {
      pad.set_action(DrivingAction::START);
      AINFO << "sending start action command.";
    }
    apollo::common::util::FillHeader("terminal", &pad);
    pad_writer_->Write(pad);
    AINFO << "send pad_message OK";
  }

  void on_chassis(const Chassis &chassis) {
    static bool is_first_emergency_mode = true;
    static int64_t count_start = 0;
    static bool waiting_reset = false;

    // check if chassis enter security mode, if enter, after 10s should reset to
    // manual
    if (chassis.driving_mode() == Chassis::EMERGENCY_MODE) {
      if (is_first_emergency_mode) {
        count_start = absl::ToUnixMicros(Clock::Now());
        is_first_emergency_mode = false;
        AINFO << "detect emergency mode.";
      } else {
        int64_t diff = absl::ToUnixMicros(Clock::Now()) - count_start;
        if (diff > EMERGENCY_MODE_HOLD_TIME) {
          count_start = absl::ToUnixMicros(Clock::Now());
          waiting_reset = true;
          // send a reset command to control
          send(RESET_COMMAND);
          AINFO << "trigger to reset emergency mode to manual mode.";
        } else {
          // nothing to do
        }
      }
    } else if (chassis.driving_mode() == Chassis::COMPLETE_MANUAL) {
      if (waiting_reset) {
        is_first_emergency_mode = true;
        waiting_reset = false;
        AINFO << "emergency mode reset to manual ok.";
      }
    }
  }

  void terminal_thread_func() {
    int mode = 0;
    bool should_exit = false;
    while (std::cin >> mode) {
      switch (mode) {
        case 0:
          send(RESET_COMMAND);
          break;
        case 1:
          send(AUTO_DRIVE_COMMAND);
          break;
        case 9:
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
  const int ROS_QUEUE_SIZE = 1;
  const int RESET_COMMAND = 1;
  const int AUTO_DRIVE_COMMAND = 2;
  const int EMERGENCY_MODE_HOLD_TIME = 4 * 1000000;
  std::shared_ptr<Reader<Chassis>> chassis_reader_;
  std::shared_ptr<Writer<PadMessage>> pad_writer_;
  std::shared_ptr<Node> node_;
};
}  // namespace

int main(int argc, char **argv) {
  apollo::cyber::Init("pad_terminal");
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
