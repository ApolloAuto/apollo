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

#ifndef PYTHON_WRAPPER_PY_RECORD_H_
#define PYTHON_WRAPPER_PY_RECORD_H_

#include <unistd.h>

#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include "cybertron/cybertron.h"
#include "cybertron/init.h"
#include "cybertron/message/protobuf_factory.h"
#include "cybertron/message/py_message.h"
#include "cybertron/message/raw_message.h"
#include "cybertron/proto/record.pb.h"
#include "cybertron/record/record_reader.h"
#include "cybertron/record/record_writer.h"

using ::apollo::cybertron::proto::Header;
using ::apollo::cybertron::record::RecordReader;
using ::apollo::cybertron::record::RecordFileWriter;

namespace apollo {
namespace cybertron {
namespace record {

class PyRecordReader {
 public:
  PyRecordReader() {}
  ~PyRecordReader() {}

  bool Open(const std::string& path) { return record_reader_.Open(path); }
  void Close() { record_reader_.Close(); }

  bool ReadMessage() { return record_reader_.ReadMessage(); }
  bool EndOfFile() { return record_reader_.EndOfFile(); }
  std::string CurrentMessageChannelName() {
    return record_reader_.CurrentMessageChannelName();
  }
  std::string CurrentRawMessage() {
    if (record_reader_.CurrentRawMessage()) {
      return record_reader_.CurrentRawMessage()->message;
    }
    return "";
  }
  uint64_t CurrentMessageTime() { return record_reader_.CurrentMessageTime(); }

  uint64_t GetMessageNumber(const std::string& channel_name) {
    return record_reader_.GetMessageNumber(channel_name);
  }

  std::string GetMessageType(const std::string& channel_name) {
    return record_reader_.GetMessageType(channel_name);
  }

  std::string GetProtoDesc(const std::string& channel_name) {
    return record_reader_.GetProtoDesc(channel_name);
  }

  std::string GetHeaderString() {
    std::string org_data;
    record_reader_.GetHeader().SerializeToString(&org_data);
    return org_data;
  }

 private:
  RecordReader record_reader_;
};

class PyRecordWriter {
 public:
  PyRecordWriter() {}
  ~PyRecordWriter() {}

  bool Open(const std::string& path) { return recored_writer_.Open(path); }

  void Close() { recored_writer_.Close(); }

  bool WriteChannel(const std::string& channel_str, const std::string& type,
                    const std::string& proto_desc) {
    return recored_writer_.WriteChannel(channel_str, type, proto_desc);
  }

  bool WriteMessage(const std::string& channel_name,
                    const std::string& rawmessage, uint64_t time) {
    return recored_writer_.WriteMessage(
        channel_name, std::make_shared<RawMessage>(rawmessage), time);
  }

 private:
  RecordWriter recored_writer_;
};

}  // namespace record
}  // namespace cybertron
}  // namespace apollo

#endif  // PYTHON_WRAPPER_PY_RECORD_H_
