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

#ifndef CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_PARAM_H_
#define CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_PARAM_H_

#include <cstdint>
#include <set>
#include <string>

namespace apollo {
namespace cyber {
namespace record {

struct PlayParam {
  bool is_play_all_channels = false;
  bool is_loop_playback = false;
  double play_rate = 1.0;
  uint64_t begin_time_ns = 0;
  uint64_t end_time_ns = UINT64_MAX;
  uint64_t start_time_s = 0;
  uint64_t delay_time_s = 0;
  uint32_t preload_time_s = 3;
  std::set<std::string> files_to_play;
  std::set<std::string> channels_to_play;
  std::set<std::string> black_channels;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_PARAM_H_
