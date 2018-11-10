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

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/message.h>
#include <google/protobuf/text_format.h>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include "cyber/common/log.h"
#include "cyber/record/file/record_file_base.h"
#include "cyber/record/file/section.h"
#include "cyber/time/time.h"

namespace apollo {
namespace cyber {
namespace record {

using google::protobuf::io::ZeroCopyInputStream;
using google::protobuf::io::FileInputStream;
using google::protobuf::io::CodedInputStream;

class RecordFileReader : public RecordFileBase {
 public:
  RecordFileReader();
  virtual ~RecordFileReader();
  bool Open(const std::string& path) override;
  void Close() override;
  bool Reset();
  bool ReadSection(Section* section);
  bool SkipSection(uint64_t size);
  template <typename T>
  bool ReadSection(uint64_t size, T* message);
  bool ReadIndex();

 private:
  bool ReadHeader();
};

template <typename T>
bool RecordFileReader::ReadSection(uint64_t size, T* message) {
  static int BUF_SIZE = 1024 * 1024;
  if (size > INT_MAX) {
    AERROR << "Size is larger than " << INT_MAX;
    return false;
  }
  uint64_t pos = CurrentPosition();
  if (size > BUF_SIZE) {
    ZeroCopyInputStream* raw_input = new FileInputStream(fd_);
    CodedInputStream* coded_input = new CodedInputStream(raw_input);
    CodedInputStream::Limit limit =
        coded_input->PushLimit(static_cast<int>(size));
    if (!message->ParseFromCodedStream(coded_input)) {
      AERROR << "Parse section message failed.";
      return false;
    }
    if (!coded_input->ConsumedEntireMessage()) {
      AERROR << "Do not consumed entire message.";
      return false;
    }
    coded_input->PopLimit(limit);
    if (message->ByteSize() != size) {
      AERROR << "Message size is not consistent in section header"
             << ", expect: " << size << ", actual: " << message->ByteSize();
      return false;
    }
    delete coded_input;
    delete raw_input;
  } else {
    char buf[BUF_SIZE];
    ssize_t count = read(fd_, buf, static_cast<ssize_t>(size));
    if (count < 0) {
      AERROR << "Read fd failed, fd_: " << fd_ << ", errno: " << errno;
      return false;
    }
    if (count != size) {
      AERROR << "Read fd failed, fd_: " << fd_ << ", expect count: " << size
             << ", actual count: " << count;
      return false;
    }
    T msg;
    if (!msg.ParseFromString(std::string(buf, count))) {
      AERROR << "Failed to parse section info.";
      return false;
    }
    message->Swap(&msg);
  }
  SetPosition(pos + size);
  return true;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_RECORD_FILE_RECORD_FILE_READER_H_
