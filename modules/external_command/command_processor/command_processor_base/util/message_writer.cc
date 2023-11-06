/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

/**
 * @file message_writer.cc
 */

#include "modules/external_command/command_processor/command_processor_base/util/message_writer.h"

#include "cyber/timer/timer.h"

namespace apollo {
namespace external_command {

MessageWriter::MessageWriter()
    : node_(apollo::cyber::CreateNode("message_writer")) {
  // Send history in 1 Hz.
  const uint32_t period = 1000;
  timer_.reset(new apollo::cyber::Timer(
      period,  //
      [this]() {
        for (auto history_writer : this->history_writers_) {
          history_writer->WriteLastMessage();
        }
      },
      false));
  timer_->Start();
}

}  // namespace external_command
}  // namespace apollo
