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

#ifndef CYBERTRON_BENCHMARK_COMPONENT_TEST_COMPONENT_TEST_H
#define CYBERTRON_BENCHMARK_COMPONENT_TEST_COMPONENT_TEST_H

#include <vector>
#include <map>
#include "data_pool.h"
#include "whisper_common.h"

namespace apollo {
namespace cybertron {

template <typename T>
class Writer;

}
}

using apollo::cybertron::Node;
using apollo::cybertron::Writer;
using apollo::cybertron::proto::CybertronWhisper;

class ComponentTest {
 protected:
  int64_t _seq = 0;
  int64_t _length = 1024;
  int64_t _length_interval = 0;
  int64_t _length_step = 0;
  int64_t _start_wait_sec = 0;
  int64_t _end_wait_sec = 0;
  int64_t _usleep_time = 0;
  int64_t _usleep_interval = 0;
  int64_t _usleep_step = 0;
  int64_t _num_messages = 128;
  std::string _name;
  std::string _configfile;

  std::vector<std::string> _in_channel_names;
  std::vector<std::string> _out_channel_names;
  std::map<std::string, std::shared_ptr<WhisperCommon>> _whispers;
  std::shared_ptr<DataPool> _data_pool;
  std::map<std::string, std::shared_ptr<Writer<CybertronWhisper>>> _writers;
  std::shared_ptr<Node> _node = nullptr;

  bool Init(const std::shared_ptr<Node>& node, const std::string& path);
};

#endif  // CYBERTRON_BENCHMARK_COMPONENT_TEST_COMPONENT_TEST_H
