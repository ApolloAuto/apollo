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

#ifndef CYBERTRON_TOOLS_CYBER_RECORDER_INFO_H_
#define CYBERTRON_TOOLS_CYBER_RECORDER_INFO_H_

#include <chrono>
#include <iomanip>
#include <string>
#include "cybertron/common/time_conversion.h"
#include "cybertron/proto/record.pb.h"
#include "cybertron/record/record_file.h"

using ::apollo::cybertron::common::UnixSecondsToString;

namespace apollo {
namespace cybertron {
namespace record {

class Info {
 public:
  Info();
  ~Info();
  bool Display(const std::string& file, bool all_sections);

 private:
  bool DisplayIndex();
  Index index_;
};

}  // namespace record
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_TOOLS_CYBER_RECORDER_INFO_H_
