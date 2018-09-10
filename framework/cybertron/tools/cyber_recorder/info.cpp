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

#include "cybertron/tools/cyber_recorder/info.h"

namespace apollo {
namespace cybertron {
namespace record {

Info::Info() {}

Info::~Info() {}

bool Info::Display(const std::string& file, bool all_sections) {
  RecordFileReader infileopt;
  if (!infileopt.Open(file)) {
    AERROR << "open record file error. file: " << file;
    return false;
  }
  if (!infileopt.ReadHeader()) {
    AERROR << "read header section of the file fail. file: " << file;
    return false;
  }
  Header header = infileopt.GetHeader();

  auto begin_time_s = header.begin_time() / 1e9;
  auto end_time_s = header.end_time() / 1e9;
  auto begin_time_str = UnixSecondsToString(begin_time_s);
  auto end_time_str = UnixSecondsToString(end_time_s);

  if (header.end_time() < header.begin_time()) {
    std::cout << "header time invalid, please reindex_.first." << std::endl;
    std::cout << "recorder begin_time: " << header.begin_time()
              << ", end_time: " << header.end_time() << std::endl;
    return false;
  }

  if (all_sections) {
    std::cout << "************** header section ******************"
              << std::endl;
  }

  auto duration_s = end_time_s - begin_time_s;

  std::cout << setiosflags(std::ios::left);
  std::cout << setiosflags(std::ios::fixed);
  unsigned int field_width = 15;
  std::cout << std::setw(field_width) << "path:" << file << std::endl
            << std::setw(field_width) << "version:" << header.major_version()
            << "." << header.minor_version() << std::endl
            << std::setw(field_width) << "duration:" << duration_s << " s"
            << std::endl
            << std::setw(field_width) << "begin_time:" << begin_time_str
            << std::endl
            << std::setw(field_width) << "end_time:" << end_time_str
            << std::endl
            << std::setw(field_width)
            << "message_number:" << header.message_number() << std::endl
            << std::setw(field_width)
            << "chunk_number:" << header.chunk_number() << std::endl
            << std::setw(field_width)
            << "index_position:" << header.index_position() << std::endl
            << std::setw(field_width) << "size:" << header.size() << " bytes";

  if (header.size() >= (1024 * 1024 * 1024)) {
    std::cout << " (" << header.size() / (1024 * 1024 * 1024.0) << " GB)";
  } else if (header.size() >= (1024 * 1024)) {
    std::cout << " (" << header.size() / (1024 * 1024.0) << " MB)";
  } else if (header.size() >= 1024) {
    std::cout << " (" << header.size() / 1024.0 << " KB)";
  }
  std::cout << std::endl;

  if (!all_sections) {
    return true;
  }
  std::cout << "******************** all sections **************" << std::endl;

  std::cout << "header|major_version=" << header.major_version()
            << "|minor_version=" << header.minor_version()
            << "|compress=" << header.compress()
            << "|chunk_interval=" << header.chunk_interval()
            << "|segment_interval=" << header.segment_interval()
            << "|index_position=" << header.index_position()
            << "|chunk_number=" << header.chunk_number()
            << "|channel_number=" << header.channel_number()
            << "|begin_time=" << header.begin_time()
            << "|end_time=" << header.end_time()
            << "|message_number=" << header.message_number()
            << "|size=" << header.size()
            << "|is_complete=" << header.is_complete()
            << "|chunk_raw_size=" << header.chunk_raw_size()
            << "|segment_raw_size=" << header.segment_raw_size() << std::endl;

  Section section;
  while (infileopt.ReadSection(&section)) {
    switch (section.type) {
      case SectionType::SECTION_CHANNEL: {
        Channel channel;
        if (!infileopt.ReadSection<Channel>(section.size, &channel)) {
          AERROR << "read message fail.";
          return false;
        }
        std::cout << "channel|name=" << channel.name()
                  << "|message_type=" << channel.message_type()
                  << "|proto_desc=..." << std::endl;
        break;
      }
      case SectionType::SECTION_CHUNK_HEADER: {
        ChunkHeader chunk_header;
        if (!infileopt.ReadSection<ChunkHeader>(section.size, &chunk_header)) {
          AERROR << "read message fail.";
          return false;
        }
        std::cout << "chunk_header|message_number="
                  << chunk_header.message_number()
                  << "|begin_time=" << chunk_header.begin_time()
                  << "|end_time=" << chunk_header.end_time()
                  << "|raw_size=" << chunk_header.raw_size() << std::endl;
        break;
      }
      case SectionType::SECTION_CHUNK_BODY: {
        ChunkBody chunk_body;
        if (!infileopt.ReadSection<ChunkBody>(section.size, &chunk_body)) {
          AERROR << "read message fail.";
          return false;
        }
        std::cout << "chunk_body|messages=..." << std::endl;
        break;
      }
      case SectionType::SECTION_INDEX: {
        if (!infileopt.ReadSection<Index>(section.size, &index_)) {
          AERROR << "read message fail.";
          return false;
        }
        DisplayIndex();
        break;
      }
      default:
        break;
    }
  }
  std::cout << "************** end of file *********************" << std::endl;
  return true;
}

bool Info::DisplayIndex() {
  for (int i = 0; i < index_.indexes_size(); i++) {
    SingleIndex* single_index = index_.mutable_indexes(i);
    std::cout << "index|postion=" << single_index->position() << "|type=";
    switch (single_index->type()) {
      case SectionType::SECTION_CHUNK_HEADER: {
        ChunkHeaderCache* chunk_header_cache =
            single_index->mutable_chunk_header_cache();
        std::cout << "chunk_header|message_number="
                  << chunk_header_cache->message_number()
                  << "|begin_time=" << chunk_header_cache->begin_time()
                  << "|end_time=" << chunk_header_cache->end_time()
                  << "|raw_size=" << chunk_header_cache->raw_size()
                  << std::endl;
        break;
      }
      case SectionType::SECTION_CHUNK_BODY: {
        ChunkBodyCache* chunk_body_cache =
            single_index->mutable_chunk_body_cache();
        std::cout << "chunk_body|message_number="
                  << chunk_body_cache->message_number() << std::endl;
        break;
      }
      case SectionType::SECTION_CHANNEL: {
        ChannelCache* channel_cache = single_index->mutable_channel_cache();
        std::cout << "channel|message_number="
                  << channel_cache->message_number()
                  << "|name=" << channel_cache->name()
                  << "|message_type=" << channel_cache->message_type()
                  << "|proto_desc=bytes" << std::endl;
        break;
      }
      default: {
        std::cout << "unknown type";
        break;
      }
    }
  }
  return true;
}

}  // namespace record
}  // namespace cybertron
}  // namespace apollo
