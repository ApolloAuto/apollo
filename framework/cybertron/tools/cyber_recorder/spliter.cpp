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

#include "cybertron/tools/cyber_recorder/spliter.h"

namespace apollo {
namespace cybertron {
namespace record {

Spliter::Spliter(const std::string& input_file, const std::string& output_file,
                 bool all_channels, const std::vector<std::string>& channel_vec,
                 uint64_t begin_time, uint64_t end_time)
    : input_file_(input_file),
      output_file_(output_file),
      channel_vec_(channel_vec),
      all_channels_(all_channels),
      begin_time_(begin_time),
      end_time_(end_time) {}

Spliter::~Spliter() {}

bool Spliter::Proc() {
  AINFO << "split record file started.";
  if (begin_time_ >= end_time_) {
    AERROR << "begin time larger or equal than end time, begin_time_: "
           << begin_time_ << "end_time_: " << end_time_;
    return false;
  }

  // open input file
  if (!reader_.Open(input_file_)) {
    AERROR << "open input file failed, file: " << input_file_;
    return false;
  }
  if (!reader_.ReadHeader()) {
    AERROR << "read input file header fail, file: " << input_file_;
    return false;
  }
  Header header = reader_.GetHeader();
  if (begin_time_ > header.end_time() || end_time_ < header.begin_time()) {
    AERROR << "time range " << begin_time_ << " to " << end_time_
           << " is not include in this record file.";
    return false;
  }

  // open output file
  HeaderBuilder* header_builder = new HeaderBuilder();
  Header new_hdr = header_builder->GetHeader();
  delete header_builder;
  if (!writer_.Open(output_file_)) {
    AERROR << "open output file failed. file: " << output_file_;
    return false;
  }
  if (!writer_.WriteHeader(new_hdr)) {
    AERROR << "write header to output file failed. file: " << output_file_;
    return false;
  }

  // read through record file
  bool skip_next_chunk_body(false);
  reader_.ReadHeader();
  while (!reader_.EndOfFile()) {
    Section section;
    if (!reader_.ReadSection(&section)) {
      AERROR << "read section failed.";
      return false;
    }
    if (section.type == SectionType::SECTION_INDEX) {
      break;
    }
    switch (section.type) {
      case SectionType::SECTION_CHANNEL: {
        Channel chan;
        if (!reader_.ReadSection<Channel>(section.size, &chan)) {
          AERROR << "read channel section fail.";
          return false;
        }
        if (all_channels_ ||
            std::find(channel_vec_.begin(), channel_vec_.end(), chan.name()) !=
                channel_vec_.end()) {
          writer_.WriteChannel(chan);
        }
        break;
      }
      case SectionType::SECTION_CHUNK_HEADER: {
        ChunkHeader chdr;
        if (!reader_.ReadSection<ChunkHeader>(section.size, &chdr)) {
          AERROR << "read chunk header section fail.";
          return false;
        }
        if (begin_time_ > chdr.end_time() || end_time_ < chdr.begin_time()) {
          skip_next_chunk_body = true;
        }
        break;
      }
      case SectionType::SECTION_CHUNK_BODY: {
        if (skip_next_chunk_body) {
          reader_.SkipSection(section.size);
          skip_next_chunk_body = false;
          break;
        }
        ChunkBody cbd;
        if (!reader_.ReadSection<ChunkBody>(section.size, &cbd)) {
          AERROR << "read chunk body section fail.";
          return false;
        }
        for (int idx = 0; idx < cbd.messages_size(); ++idx) {
          if (!all_channels_ &&
              std::find(channel_vec_.begin(), channel_vec_.end(),
                        cbd.messages(idx).channel_name()) ==
                  channel_vec_.end()) {
            continue;
          }
          if (cbd.messages(idx).time() < begin_time_ ||
              cbd.messages(idx).time() > end_time_) {
            continue;
          }
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
        break;
      }
    }  // end for switch
  }    // end for while
  AINFO << "split record file done.";
  return true;
}  // end for Proc()

}  // namespace record
}  // namespace cybertron
}  // namespace apollo
