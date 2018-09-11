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

#include "cybertron/tools/cyber_recorder/recoverer.h"

namespace apollo {
namespace cybertron {
namespace record {

Recoverer::Recoverer(const std::string& input_file,
                     const std::string& output_file)
    : writer_(nullptr), input_file_(input_file), output_file_(output_file) {}

Recoverer::~Recoverer() {}

bool Recoverer::Proc() {
  if (!infileopt_.Open(input_file_)) {
    AERROR << "open input file fail, file: " << input_file_;
    return false;
  }

  // skip header
  if (!infileopt_.ReadHeader()) {
    AERROR << "read input file header fail, file: " << input_file_;
    return false;
  }
  Header header = infileopt_.GetHeader();

  writer_.reset(new RecordWriter());
  if (!writer_->Open(output_file_)) {
    AERROR << "open output file fail, file: " << output_file_;
    return false;
  }

  if (!infileopt_.ReadIndex()) {
    AINFO << "read input file index fail, file: " << input_file_;
    AINFO << "all information in index section will lost";
  } else {
    Index index = infileopt_.GetIndex();
    for (int i = 0; i < index.indexes_size(); i++) {
      SingleIndex* single_index = index.mutable_indexes(i);
      if (single_index->type() != SectionType::SECTION_CHANNEL) {
        continue;
      }
      ChannelCache* channel_cache = single_index->mutable_channel_cache();
      if (std::find(channel_vec_.begin(), channel_vec_.end(),
                    channel_cache->name()) == channel_vec_.end()) {
        channel_vec_.push_back(channel_cache->name());
        writer_->WriteChannel(channel_cache->name(),
                              channel_cache->message_type(),
                              channel_cache->proto_desc());
      }
    }
  }

  // sequential reading sections after header
  Section section;
  while (!infileopt_.EndOfFile()) {
    if (!infileopt_.ReadSection(&section)) {
      AINFO << "read section fail, try next.";
      continue;
    }
    if (section.type == SectionType::SECTION_INDEX) {
      break;
    }
    switch (section.type) {
      case SectionType::SECTION_CHANNEL: {
        Channel channel;
        if (!infileopt_.ReadSection<Channel>(section.size, &channel)) {
          AINFO << "one channel section broken, skip it.";
        } else {
          if (std::find(channel_vec_.begin(), channel_vec_.end(),
                        channel.name()) == channel_vec_.end()) {
            channel_vec_.push_back(channel.name());
            writer_->WriteChannel(channel.name(), channel.message_type(),
                                  channel.proto_desc());
          }
        }
        break;
      }
      case SectionType::SECTION_CHUNK_HEADER: {
        ChunkHeader chunk_header;
        if (!infileopt_.ReadSection<ChunkHeader>(section.size, &chunk_header)) {
          AINFO << "one chunk header section broken, skip it.";
        }
        break;
      }
      case SectionType::SECTION_CHUNK_BODY: {
        ChunkBody chunk_body;
        if (!infileopt_.ReadSection<ChunkBody>(section.size, &chunk_body)) {
          AINFO << "one chunk body section broken, skip it";
          break;
        }
        for (int idx = 0; idx < chunk_body.messages_size(); ++idx) {
          if (!writer_->WriteMessage(chunk_body.messages(idx))) {
            AERROR << "datafile_->Write() fail.";
            return false;
          }
        }
        break;
      }
      default: {
        AERROR << "this section should not be here, section type: "
               << section.type;
        break;
      }
    }  // end for switch
  }    // end for while
  AINFO << "recover record file done";
  return true;
}  // end for Proc()

}  // namespace record
}  // namespace cybertron
}  // namespace apollo
