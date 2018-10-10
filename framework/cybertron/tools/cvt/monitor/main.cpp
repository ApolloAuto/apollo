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

#include "cybertron_topology_message.h"
#include "general_channel_message.h"

#include "channel_msg_factory.h"
#include "screen.h"

#include "cybertron/init.h"
#include "cybertron/service_discovery/topology_manager.h"

#include <signal.h>
#include <iostream>

namespace {
void SigResizeHandle(int) { Screen::Instance()->Resize(); }
void SigCtrlCHandle(int) { Screen::Instance()->Stop(); }

void printHelp(const char *cmdName)
{
  std::cout << "Usage:\n"
            << cmdName << "  [option]\nOption:\n"
            << "   -h print help info\n"
            << "   -c specify one channel\n"
            << "Interactive Command:\n"
            << Screen::InteractiveCmdStr << std::endl;
}

enum COMMAND
{
  TOO_MANY_PARAMETER,
  HELP,      // 2
  NO_OPTION, // 1
  CHANNEL    // 3 -> 4
};

COMMAND parseOption(int argc, char *const argv[], std::string &commandVal)
{
  if (argc > 4)
    return TOO_MANY_PARAMETER;
  int index = 1;
  while (true)
  {
    const char *opt = argv[index];
    if (opt == nullptr)
      break;
    if (strcmp(opt, "-h") == 0)
      return HELP;
    if (strcmp(opt, "-c") == 0)
    {
      if (argv[index + 1])
      {
        commandVal = argv[index + 1];
        return CHANNEL;
      }
    }

    ++index;
  }

  return NO_OPTION;
}

} // namespace

int main(int argc, char *argv[])
{
  std::string val;

  COMMAND com = parseOption(argc, argv, val);

  switch (com)
  {
  case TOO_MANY_PARAMETER:
    std::cout << "Too many paramtes\n";
  case HELP:
    printHelp(argv[0]);
    return 0;
  default:;
  }

  apollo::cybertron::Init(argv[0]);
  FLAGS_minloglevel = 3;
  FLAGS_alsologtostderr = 0;
  FLAGS_colorlogtostderr = 0;

  ChannelMsgFactory *f = ChannelMsgFactory::Instance();
  f->RegisterChildFactory("apollo::cybertron::message::RawMessage",
                          GeneralChannelMessage::Instance);
  f->SetDefaultChildFactory("apollo::cybertron::message::RawMessage");

  CybertronTopologyMessage topologyMsg(val);

  auto topologyCallback =
      [&topologyMsg](const apollo::cybertron::proto::ChangeMsg &change_msg) {
        topologyMsg.TopologyChanged(change_msg);
      };

  apollo::cybertron::service_discovery::TopologyManager::Instance()
      ->channel_manager()
      ->AddChangeListener(topologyCallback);

  Screen *s = Screen::Instance();

  signal(SIGWINCH, SigResizeHandle);
  signal(SIGINT, SigCtrlCHandle);

  s->SetCurrentRenderMessage(&topologyMsg);

  s->Init();
  s->Run();

  return 0;
}
