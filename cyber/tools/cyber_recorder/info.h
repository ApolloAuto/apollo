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

#ifndef CYBER_TOOLS_CYBER_RECORDER_INFO_H_
#define CYBER_TOOLS_CYBER_RECORDER_INFO_H_

#include <chrono>
#include <iomanip>
#include <string>

#include "cyber/common/time_conversion.h"
#include "cyber/proto/record.pb.h"
#include "cyber/record/file/record_file_reader.h"

using ::apollo::cyber::common::UnixSecondsToString;

namespace apollo {
namespace cyber {
namespace record {

class Info {
 public:
  Info();
  ~Info();
  bool Display(const std::string& file);
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TOOLS_CYBER_RECORDER_INFO_H_
