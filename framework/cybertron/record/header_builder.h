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

#ifndef CYBERTRON_RECORD_HEADER_BUILDER_H_
#define CYBERTRON_RECORD_HEADER_BUILDER_H_

#include "cybertron/proto/record.pb.h"

using ::apollo::cybertron::proto::Header;
using ::apollo::cybertron::proto::CompressType;

namespace apollo {
namespace cybertron {
namespace record {

class HeaderBuilder {
 public:
  HeaderBuilder();
  virtual ~HeaderBuilder();
  void BuildDefault();
  void BuildSegmentPart(uint64_t segment_interval, uint64_t segment_raw_size);
  void BuildChunkPart(uint64_t chunk_interval, uint64_t chunk_raw_size);
  const Header GetHeader();

 private:
  Header header_;
  const uint32_t major_version_ = 1;
  const uint32_t minor_version_ = 0;
  const CompressType compress_type_ = CompressType::COMPRESS_NONE;
  const uint64_t chunk_interval_ = 20 * 1e9L;                 // 20s
  const uint64_t segment_interval_ = 60 * 1e9L;               // 60s
  const uint64_t chunk_raw_size_ = 200 * 1024 * 1024;         // 200MB
  const uint64_t segment_raw_size_ = 2 * 1024 * 1024 * 1024;  // 2GB
};

}  // namespace record
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_RECORD_HEADER_BUILDER_H_
