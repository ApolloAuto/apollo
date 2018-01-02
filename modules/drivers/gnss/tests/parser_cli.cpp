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

// A command-line interface (CLI) tool of parser. It parses a binary log file.
// It is supposed to be
// used for verifying if the parser works properly.

#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <memory>

#include "gnss/parser.h"
#include "gnss/stream.h"

namespace apollo {
namespace drivers {
namespace gnss {

constexpr size_t BUFFER_SIZE = 128;

void Parse(const char* filename, char parser_type) {
  std::ios::sync_with_stdio(false);
  std::ifstream f(filename, std::ifstream::binary);
  char b[BUFFER_SIZE];
  std::unique_ptr<Parser> p;
  switch (parser_type) {
    case 'n':
      p.reset(Parser::create_novatel());
      break;
    default:
      std::cout << "Log type should be either 'n' or 'u'" << std::endl;
      return;
  }
  while (f) {
    f.read(b, BUFFER_SIZE);
    p->update(reinterpret_cast<uint8_t*>(b), f.gcount());
    for (;;) {
      MessagePtr msg_ptr;
      if (p->get_message(msg_ptr) == Parser::MessageType::NONE) {
        break;
      }
      // msg_ptr->PrintDebugString();
    }
  }
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cout << "Usage: " << argv[0] << " filename [n|u]" << std::endl;
    return 0;
  }

  ros::Time::init();
  ::apollo::drivers::gnss::Parse(argv[1], argv[2][0]);
  return 0;
}
