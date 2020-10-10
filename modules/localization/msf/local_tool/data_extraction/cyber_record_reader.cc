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

#include "modules/localization/msf/local_tool/data_extraction/cyber_record_reader.h"

#include "cyber/cyber.h"
#include "cyber/record/record_reader.h"

namespace apollo {
namespace localization {
namespace msf {

using cyber::record::RecordReader;

CyberRecordReader::CyberRecordReader() {}

CyberRecordReader::~CyberRecordReader() {}

void CyberRecordReader::Subscribe(
    const std::string &topic,
    const std::function<void(const std::string &)> call_back) {
  call_back_map_[topic] = call_back;
  topics_.push_back(topic);
}

void CyberRecordReader::Read(const std::string &file_name) {
  RecordReader reader(file_name);
  cyber::record::RecordMessage message;
  while (reader.ReadMessage(&message)) {
    auto itr = call_back_map_.find(message.channel_name);
    if (itr != call_back_map_.end()) {
      itr->second(message.content);
    }
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
