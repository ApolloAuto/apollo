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

#include "cyber/record/header_builder.h"

namespace apollo {
namespace cyber {
namespace record {

using ::apollo::cyber::proto::CompressType;

proto::Header HeaderBuilder::GetHeader() {
  proto::Header header;
  header.set_major_version(MAJOR_VERSION_);
  header.set_minor_version(MINOR_VERSION_);
  header.set_compress(COMPRESS_TYPE_);
  header.set_chunk_interval(CHUNK_INTERVAL_);
  header.set_segment_interval(SEGMENT_INTERVAL_);
  header.set_index_position(0);
  header.set_chunk_number(0);
  header.set_channel_number(0);
  header.set_begin_time(0);
  header.set_end_time(0);
  header.set_message_number(0);
  header.set_size(0);
  header.set_is_complete(false);
  header.set_chunk_raw_size(CHUNK_RAW_SIZE_);
  header.set_segment_raw_size(SEGMENT_RAW_SIZE_);
  return header;
}

proto::Header HeaderBuilder::GetHeaderWithSegmentParams(
    const uint64_t segment_interval, const uint64_t segment_raw_size) {
  proto::Header header;
  header.set_major_version(MAJOR_VERSION_);
  header.set_minor_version(MINOR_VERSION_);
  header.set_compress(COMPRESS_TYPE_);
  header.set_chunk_interval(CHUNK_INTERVAL_);
  header.set_chunk_raw_size(CHUNK_RAW_SIZE_);
  header.set_index_position(0);
  header.set_chunk_number(0);
  header.set_channel_number(0);
  header.set_begin_time(0);
  header.set_end_time(0);
  header.set_message_number(0);
  header.set_size(0);
  header.set_is_complete(false);
  header.set_segment_raw_size(segment_raw_size);
  header.set_segment_interval(segment_interval);
  return header;
}

proto::Header HeaderBuilder::GetHeaderWithChunkParams(
    const uint64_t chunk_interval, const uint64_t chunk_raw_size) {
  proto::Header header;
  header.set_major_version(MAJOR_VERSION_);
  header.set_minor_version(MINOR_VERSION_);
  header.set_compress(COMPRESS_TYPE_);
  header.set_segment_interval(SEGMENT_INTERVAL_);
  header.set_segment_raw_size(SEGMENT_RAW_SIZE_);
  header.set_index_position(0);
  header.set_chunk_number(0);
  header.set_channel_number(0);
  header.set_begin_time(0);
  header.set_end_time(0);
  header.set_message_number(0);
  header.set_size(0);
  header.set_is_complete(false);
  header.set_chunk_interval(chunk_interval);
  header.set_chunk_raw_size(chunk_raw_size);
  return header;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
