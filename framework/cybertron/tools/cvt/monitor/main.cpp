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

#include "messages/cybertron_topology_message.h"
#include "messages/unknown_message.h"

#include "channel_msg_factory.h"
#include "screen.h"

#include "cybertron/init.h"
#include "cybertron/service_discovery/topology_manager.h"

#include <signal.h>
#include <iostream>

namespace {
void SigResizeHandle(int) { Screen::Instance()->Resize(); }

void printHelp(const char* cmdName) {
  std::cout << "Usage:\n"
            << cmdName << "  run this tool\n"
            << "   -h print help info\n"
            << "\nInteractive Command:\n"
            << Screen::InteractiveCmdStr << std::endl;
}
}  // namespace

int main(int argc, char* argv[]) {
  if (argc > 2) {
    std::cout << "Too many parameters\n";
    printHelp(argv[0]);
    return 0;
  } else if (argc == 2) {
    if (strcmp(argv[1], "-h") == 0) {
      printHelp(argv[0]);
      return 0;
    }
  }

  apollo::cybertron::Init(argv[0]);
  FLAGS_minloglevel = 3;
  FLAGS_alsologtostderr = 0;
  FLAGS_colorlogtostderr = 0;

  CybertronChannelMsgFactory* f = CybertronChannelMsgFactory::Instance();
  f->RegisterChildFactory("apollo::cybertron::message::RawMessage",
                          UnknownMessage::Instance);
  f->SetDefaultChildFactory("apollo::cybertron::message::RawMessage");

  CybertronTopologyMessage topologyMsg;

  auto topologyCallback =
      [&topologyMsg](const apollo::cybertron::proto::ChangeMsg& change_msg) {
        topologyMsg.TopologyChanged(change_msg);
      };

  apollo::cybertron::service_discovery::TopologyManager::Instance()
      ->channel_manager()
      ->AddChangeListener(topologyCallback);

  Screen* s = Screen::Instance();

  signal(SIGWINCH, SigResizeHandle);

  s->SetCurrentRenderMessage(&topologyMsg);

  s->Init();
  s->Run();

  return 0;
}
