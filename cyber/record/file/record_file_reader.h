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

#ifndef CYBER_RECORD_FILE_RECORD_FILE_READER_H_
#define CYBER_RECORD_FILE_RECORD_FILE_READER_H_

#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include "cyber/common/log.h"
#include "cyber/record/file/record_file_base.h"
#include "cyber/record/file/section.h"

namespace apollo {
namespace cyber {
namespace record {

class RecordFileReader : public RecordFileBase {
 public:
  RecordFileReader();
  virtual ~RecordFileReader();
  bool Open(const std::string& path) override;
  void Close() override;
  bool ReadSection(Section* section);
  void SkipSection(uint64_t size, uint64_t fixed_size = 0);
  template <typename T>
  bool ReadSection(uint64_t size, T* message, uint64_t fixed_size = 0);
  bool ReadHeader();
  bool ReadIndex();
  bool EndOfFile();
  void Reset();

 private:
  std::ifstream ifstream_;
};

template <typename T>
bool RecordFileReader::ReadSection(uint64_t size, T* message,
                                   uint64_t fixed_size) {
  if (size == 0) {
    AERROR << "size is zero.";
    return false;
  }
  std::string str;
  str.resize(size);
  if (0 < fixed_size && fixed_size < size) {
    AERROR << "size is larger than fixed size, size: " << size
           << ", fixed size: " << fixed_size;
    return false;
  }
  int64_t backup_position = ifstream_.tellg();
  ifstream_.read(reinterpret_cast<char*>(const_cast<char*>(str.c_str())), size);
  if (ifstream_.gcount() != size) {
    AERROR << "read section message fail, expect size: " << size
           << ", actual size: " << ifstream_.gcount();
    return false;
  }
  if (fixed_size > size) {
    ifstream_.seekg(backup_position + fixed_size, std::ios::beg);
  }
  T msg;
  if (!msg.ParseFromString(str)) {
    AERROR << "Failed to parse section info.";
    return false;
  }
  message->Swap(&msg);
  return true;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_RECORD_FILE_RECORD_FILE_READER_H_
