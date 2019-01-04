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

#ifndef CYBER_RECORD_RECORD_READER_H_
#define CYBER_RECORD_RECORD_READER_H_

#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include "cyber/proto/record.pb.h"
#include "cyber/record/file/record_file_reader.h"
#include "cyber/record/record_base.h"
#include "cyber/record/record_message.h"

namespace apollo {
namespace cyber {
namespace record {

class RecordReader : public RecordBase {
 public:
  using FileReaderPtr = std::unique_ptr<RecordFileReader>;
  using ChannelInfoMap = std::unordered_map<std::string, proto::ChannelCache>;

  explicit RecordReader(const std::string& file);
  virtual ~RecordReader();

  bool IsValid() const { return is_valid_; }

  bool ReadMessage(RecordMessage* message, uint64_t begin_time = 0,
                   uint64_t end_time = UINT64_MAX);
  void Reset();

  uint64_t GetMessageNumber(const std::string& channel_name) const override;

  const std::string& GetMessageType(
      const std::string& channel_name) const override;

  const std::string& GetProtoDesc(
      const std::string& channel_name) const override;

  std::set<std::string> GetChannelList() const;

  const proto::Header& header() const { return header_; }
  const ChannelInfoMap& channel_info() const { return channel_info_; }

 private:
  bool ReadNextChunk(uint64_t begin_time, uint64_t end_time);

  bool is_valid_ = false;
  bool reach_end_ = false;
  proto::ChunkBody chunk_;
  proto::Index index_;
  int message_index_ = 0;
  ChannelInfoMap channel_info_;
  FileReaderPtr file_reader_;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_RECORD_RECORD_READER_H_
