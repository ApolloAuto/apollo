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

#include "cyber/tools/cyber_recorder/player/terminal_controller.h"

#include <stdio.h>

namespace apollo {
namespace cyber {
namespace record {

TerminalController::TerminalController() : max_fd_(0) {}

TerminalController::~TerminalController() {}

void TerminalController::SetUp() {
  const int fd = fileno(stdin);
  tcgetattr(fd, &original_flags_);
  termios flags = original_flags_;

  flags.c_lflag &= ~ICANON;  // set raw (unset canonical modes)
  flags.c_cc[VMIN] = 0;   // min 1 char for blocking, 0 chars for non-blocking
  flags.c_cc[VTIME] = 0;  // block if waiting for char

  tcsetattr(fd, TCSANOW, &flags);
  FD_ZERO(&stdin_fdset_);
  FD_SET(fd, &stdin_fdset_);
  max_fd_ = fd + 1;
}

void TerminalController::TearDown() {
  const int fd = fileno(stdin);
  tcsetattr(fd, TCSANOW, &original_flags_);
}

char TerminalController::ReadChar() {
  fd_set select_fd_set = stdin_fdset_;
  timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  if (select(max_fd_, &select_fd_set, nullptr, nullptr, &tv) <= 0) {
    return EOF;
  }
  auto ch = getc(stdin);
  return ch;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
