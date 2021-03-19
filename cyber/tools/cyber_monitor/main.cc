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

#include <csignal>
#include <iostream>

#include "cyber/init.h"
#include "cyber/service_discovery/topology_manager.h"
#include "cyber/tools/cyber_monitor/cyber_topology_message.h"
#include "cyber/tools/cyber_monitor/general_channel_message.h"
#include "cyber/tools/cyber_monitor/screen.h"

namespace {
void SigResizeHandle(int) { Screen::Instance()->Resize(); }
void SigCtrlCHandle(int) { Screen::Instance()->Stop(); }

void printHelp(const char *cmd_name) {
  std::cout << "Usage:\n"
            << cmd_name << "  [option]\nOption:\n"
            << "   -h print help info\n"
            << "   -c specify one channel\n"
            << "Interactive Command:\n"
            << Screen::InteractiveCmdStr << std::endl;
}

enum COMMAND {
  TOO_MANY_PARAMETER,
  HELP,       // 2
  NO_OPTION,  // 1
  CHANNEL     // 3 -> 4
};

COMMAND ParseOption(int argc, char *const argv[], std::string *command_val) {
  if (argc > 4) {
    return TOO_MANY_PARAMETER;
  }
  int index = 1;
  while (true) {
    const char *opt = argv[index];
    if (opt == nullptr) {
      break;
    }
    if (strcmp(opt, "-h") == 0) {
      return HELP;
    }
    if (strcmp(opt, "-c") == 0) {
      if (argv[index + 1]) {
        *command_val = argv[index + 1];
        return CHANNEL;
      }
    }

    ++index;
  }

  return NO_OPTION;
}

}  // namespace

int main(int argc, char *argv[]) {
  std::string val;

  COMMAND com = ParseOption(argc, argv, &val);

  switch (com) {
    case TOO_MANY_PARAMETER:
      std::cout << "Too many paramtes\n";
    case HELP:
      printHelp(argv[0]);
      return 0;
    default: {
    }
  }

  apollo::cyber::Init(argv[0]);
  FLAGS_minloglevel = 3;
  FLAGS_alsologtostderr = 0;
  FLAGS_colorlogtostderr = 0;

  CyberTopologyMessage topology_msg(val);

  auto topology_callback =
      [&topology_msg](const apollo::cyber::proto::ChangeMsg &change_msg) {
        topology_msg.TopologyChanged(change_msg);
      };

  auto channel_manager =
      apollo::cyber::service_discovery::TopologyManager::Instance()
          ->channel_manager();
  channel_manager->AddChangeListener(topology_callback);

  std::vector<apollo::cyber::proto::RoleAttributes> role_vec;
  channel_manager->GetWriters(&role_vec);
  for (auto &role : role_vec) {
    topology_msg.AddReaderWriter(role, true);
  }

  role_vec.clear();
  channel_manager->GetReaders(&role_vec);
  for (auto &role : role_vec) {
    topology_msg.AddReaderWriter(role, false);
  }

  Screen *s = Screen::Instance();

  signal(SIGWINCH, SigResizeHandle);
  signal(SIGINT, SigCtrlCHandle);

  s->SetCurrentRenderMessage(&topology_msg);

  s->Init();
  s->Run();

  return 0;
}
