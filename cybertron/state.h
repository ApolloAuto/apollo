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

#ifndef CYBERTRON_STATE_H_
#define CYBERTRON_STATE_H_

#include <cstdint>

namespace apollo {
namespace cybertron {

enum State : std::uint8_t {
  STATE_UNINITIALIZED = 0,
  STATE_INITIALIZED,
  STATE_SHUTTING_DOWN,
  STATE_SHUTDOWN,
};

State GetState();
void SetState(const State& state);

bool OK();
bool IsShutdown();
void WaitForShutdown();

}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_STATE_H_
