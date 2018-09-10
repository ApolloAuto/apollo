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
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include "cybertron/cybertron.h"
#include "cybertron/data/data_fusion.h"
#include "cybertron/data/meta_data.h"
#include "cybertron/init.h"
#include "cybertron/message/protobuf_factory.h"
#include "cybertron/message/py_message.h"
#include "cybertron/message/raw_message.h"
#include "cybertron/record/header_builder.h"
#include "cybertron/record/record_file.h"
#include "cybertron/proto/record.pb.h"

using ::apollo::cybertron::proto::Header;
using ::apollo::cybertron::proto::Channel;
using ::apollo::cybertron::proto::SingleMessage;
using ::apollo::cybertron::record::RecordFileReader;
using ::apollo::cybertron::record::RecordFileWriter;

namespace apollo {
namespace cybertron {
namespace record {

bool py_is_shutdown() { return cybertron::IsShutdown(); }
bool py_init() {
  static bool inited = false;

  if (inited) {
    AINFO << "cybertron already inited.";
    return true;
  }

  if (!apollo::cybertron::Init("cyber_python")) {
    AINFO << "cybertron::Init failed.";
    return false;
  }
  inited = true;
  AINFO << "cybertron init succ.";
  return true;
}

bool py_OK() { return apollo::cybertron::OK(); }

class PyRecordFileReader {
 public:
  PyRecordFileReader()  {
  }
  ~PyRecordFileReader() { }

  bool Open(const std::string &path) { return recored_file_reader_.Open(path); }
  void Close() { recored_file_reader_.Close(); }

  bool ReadHeader() { return recored_file_reader_.ReadHeader(); }
  bool ReadIndex() { return recored_file_reader_.ReadIndex(); }
  bool EndOfFile() { return recored_file_reader_.EndOfFile(); }

 private:
  RecordFileReader recored_file_reader_;
};

class PyRecordFileWriter {
 public:
  PyRecordFileWriter()  {
  }
  ~PyRecordFileWriter() { }

  bool Open(const std::string &path) { return recored_file_writer_.Open(path); }
  void Close() { recored_file_writer_.Close(); }

  bool WriteHeader(const std::string& header_str) {
    Header header;
    header.ParseFromString(header_str);
    return recored_file_writer_.WriteHeader(header); 
  }
  bool WriteChannel(const std::string& channel_str) {
    Channel channel;
    channel.ParseFromString(channel_str);
    return recored_file_writer_.WriteChannel(channel); 
  }
  bool AddSingleMessage(const std::string& single_str) { 
    SingleMessage single;
    single.ParseFromString(single_str);
    return recored_file_writer_.AddSingleMessage(single); 
  }
 private:
  RecordFileWriter recored_file_writer_;
};


}  // namespace record
}  // namespace cybertron
}  // namespace apollo

#endif  // PYTHON_WRAPPER_PY_RECORD_H_
