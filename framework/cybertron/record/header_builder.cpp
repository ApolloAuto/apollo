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

#include "cybertron/record/header_builder.h"

namespace apollo {
namespace cybertron {
namespace record {

HeaderBuilder::HeaderBuilder() { BuildDefault(); }

HeaderBuilder::~HeaderBuilder() {}

void HeaderBuilder::BuildDefault() {
  header_.set_major_version(major_version_);
  header_.set_minor_version(minor_version_);
  header_.set_compress(compress_type_);
  header_.set_chunk_interval(chunk_interval_);
  header_.set_segment_interval(segment_interval_);
  header_.set_index_position(0);
  header_.set_chunk_number(0);
  header_.set_channel_number(0);
  header_.set_begin_time(0);
  header_.set_end_time(0);
  header_.set_message_number(0);
  header_.set_size(0);
  header_.set_is_complete(false);
  header_.set_chunk_raw_size(chunk_raw_size_);
  header_.set_segment_raw_size(segment_raw_size_);
}

void HeaderBuilder::BuildSegmentPart(uint64_t segment_interval,
                                     uint64_t segment_raw_size) {
  header_.set_segment_interval(segment_interval);
  header_.set_segment_raw_size(segment_raw_size);
}

void HeaderBuilder::BuildChunkPart(uint64_t chunk_interval,
                                   uint64_t chunk_raw_size) {
  header_.set_chunk_interval(chunk_interval);
  header_.set_chunk_raw_size(chunk_raw_size);
}

const Header HeaderBuilder::GetHeader() { return header_; }

}  // namespace record
}  // namespace cybertron
}  // namespace apollo
