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

#include "cyber/tools/cyber_recorder/info.h"

#include "cyber/record/record_message.h"

namespace apollo {
namespace cyber {
namespace record {

using apollo::cyber::proto::ChannelCache;
using apollo::cyber::record::kGB;
using apollo::cyber::record::kKB;
using apollo::cyber::record::kMB;

Info::Info() {}

Info::~Info() {}

bool Info::Display(const std::string& file) {
  RecordFileReader file_reader;
  if (!file_reader.Open(file)) {
    AERROR << "open record file error. file: " << file;
    return false;
  }
  proto::Header hdr = file_reader.GetHeader();

  std::cout << setiosflags(std::ios::left);
  std::cout << setiosflags(std::ios::fixed);

  int w = 16;
  // file name
  std::cout << std::setw(w) << "record_file: " << file << std::endl;

  // version
  std::cout << std::setw(w) << "version: " << hdr.major_version() << "."
            << hdr.minor_version() << std::endl;

  // time and duration
  auto begin_time_s = static_cast<double>(hdr.begin_time()) / 1e9;
  auto end_time_s = static_cast<double>(hdr.end_time()) / 1e9;
  auto duration_s = end_time_s - begin_time_s;
  auto begin_time_str = UnixSecondsToString(static_cast<int>(begin_time_s));
  auto end_time_str = UnixSecondsToString(static_cast<int>(end_time_s));
  std::cout << std::setw(w) << "duration: " << duration_s << " Seconds"
            << std::endl;
  std::cout << std::setw(w) << "begin_time: " << begin_time_str << std::endl;
  std::cout << std::setw(w) << "end_time: " << end_time_str << std::endl;

  // size
  std::cout << std::setw(w) << "size: " << hdr.size() << " Bytes";
  if (hdr.size() >= kGB) {
    std::cout << " (" << static_cast<float>(hdr.size()) / kGB << " GB)";
  } else if (hdr.size() >= kMB) {
    std::cout << " (" << static_cast<float>(hdr.size()) / kMB << " MB)";
  } else if (hdr.size() >= kKB) {
    std::cout << " (" << static_cast<float>(hdr.size()) / kKB << " KB)";
  }
  std::cout << std::endl;

  // is_complete
  std::cout << std::setw(w) << "is_complete:";
  if (hdr.is_complete()) {
    std::cout << "true";
  } else {
    std::cout << "false";
  }
  std::cout << std::endl;

  // message_number
  std::cout << std::setw(w) << "message_number: " << hdr.message_number()
            << std::endl;

  // channel_number
  std::cout << std::setw(w) << "channel_number: " << hdr.channel_number()
            << std::endl;

  // read index section
  if (!file_reader.ReadIndex()) {
    AERROR << "read index section of the file fail. file: " << file;
    return false;
  }

  // channel info
  std::cout << std::setw(w) << "channel_info: " << std::endl;
  proto::Index idx = file_reader.GetIndex();
  for (int i = 0; i < idx.indexes_size(); ++i) {
    ChannelCache* cache = idx.mutable_indexes(i)->mutable_channel_cache();
    if (idx.mutable_indexes(i)->type() == proto::SectionType::SECTION_CHANNEL) {
      std::cout << std::setw(w) << "";
      std::cout << resetiosflags(std::ios::right);
      std::cout << std::setw(50) << cache->name();
      std::cout << setiosflags(std::ios::right);
      std::cout << std::setw(8) << cache->message_number();
      std::cout << std::setw(0) << " messages: ";
      std::cout << cache->message_type();
      std::cout << std::endl;
    }
  }
  return true;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
