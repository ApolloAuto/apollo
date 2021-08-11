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

#include "cyber/tools/cyber_recorder/recoverer.h"

#include "cyber/base/for_each.h"
#include "cyber/record/header_builder.h"

namespace apollo {
namespace cyber {
namespace record {

using apollo::cyber::proto::Channel;
using apollo::cyber::proto::ChunkHeader;
using apollo::cyber::proto::SectionType;

Recoverer::Recoverer(const std::string& input_file,
                     const std::string& output_file)
    : input_file_(input_file), output_file_(output_file) {}

Recoverer::~Recoverer() {}

bool Recoverer::Proc() {
  if (!reader_.Open(input_file_)) {
    AERROR << "open input file failed, file: " << input_file_;
    return false;
  }

  // open output file
  proto::Header new_hdr = HeaderBuilder::GetHeader();
  if (!writer_.Open(output_file_)) {
    AERROR << "open output file failed. file: " << output_file_;
    return false;
  }
  if (!writer_.WriteHeader(new_hdr)) {
    AERROR << "write header to output file failed. file: " << output_file_;
    return false;
  }

  // write channel sections
  if (reader_.ReadIndex()) {
    proto::Index index = reader_.GetIndex();
    FOR_EACH(i, 0, index.indexes_size()) {
      proto::SingleIndex* single_index = index.mutable_indexes(i);
      if (single_index->type() != SectionType::SECTION_CHANNEL) {
        continue;
      }
      ChannelCache* chan_cache = single_index->mutable_channel_cache();
      if (std::find(channel_vec_.begin(), channel_vec_.end(),
                    chan_cache->name()) == channel_vec_.end()) {
        channel_vec_.push_back(chan_cache->name());
        Channel chan;
        chan.set_name(chan_cache->name());
        chan.set_message_type(chan_cache->message_type());
        chan.set_proto_desc(chan_cache->proto_desc());
        writer_.WriteChannel(chan);
      }
    }
  }

  // read through record file
  reader_.Reset();
  while (!reader_.EndOfFile()) {
    Section section;
    if (!reader_.ReadSection(&section)) {
      AINFO << "read section failed, try next.";
      continue;
    }
    if (section.type == SectionType::SECTION_INDEX) {
      break;
    }
    switch (section.type) {
      case SectionType::SECTION_CHANNEL: {
        Channel chan;
        if (!reader_.ReadSection<Channel>(section.size, &chan)) {
          AINFO << "one channel section broken, skip it.";
          break;
        }
        if (std::find(channel_vec_.begin(), channel_vec_.end(), chan.name()) ==
            channel_vec_.end()) {
          channel_vec_.push_back(chan.name());
          writer_.WriteChannel(chan);
        }
        break;
      }
      case SectionType::SECTION_CHUNK_HEADER: {
        ChunkHeader chdr;
        if (!reader_.ReadSection<ChunkHeader>(section.size, &chdr)) {
          AINFO << "one chunk header section broken, skip it.";
        }
        break;
      }
      case SectionType::SECTION_CHUNK_BODY: {
        ChunkBody cbd;
        if (!reader_.ReadSection<ChunkBody>(section.size, &cbd)) {
          AINFO << "one chunk body section broken, skip it";
          break;
        }
        for (int idx = 0; idx < cbd.messages_size(); ++idx) {
          if (!writer_.WriteMessage(cbd.messages(idx))) {
            AERROR << "add new message failed.";
            return false;
          }
        }
        break;
      }
      default: {
        AERROR << "this section should not be here, section type: "
               << section.type;
        return false;
      }
    }  // end for switch
  }    // end for while
  AINFO << "recover record file done.";
  return true;
}  // end for Proc()

}  // namespace record
}  // namespace cyber
}  // namespace apollo
