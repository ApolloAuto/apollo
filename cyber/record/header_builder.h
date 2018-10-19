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

#ifndef CYBER_RECORD_HEADER_BUILDER_H_
#define CYBER_RECORD_HEADER_BUILDER_H_

#include "cyber/proto/record.pb.h"

namespace apollo {
namespace cyber {
namespace record {

class HeaderBuilder {
 public:
  static proto::Header GetHeaderWithSegmentParams(
      const uint64_t segment_interval, const uint64_t segment_raw_size);
  static proto::Header GetHeaderWithChunkParams(const uint64_t chunk_interval,
                                                const uint64_t chunk_raw_size);
  static proto::Header GetHeader();

 private:
  static const uint32_t MAJOR_VERSION_ = 1;
  static const uint32_t MINOR_VERSION_ = 0;
  static const proto::CompressType COMPRESS_TYPE_ =
      proto::CompressType::COMPRESS_NONE;
  static const uint64_t CHUNK_INTERVAL_ = 20 * 1e9L;                    // 20s
  static const uint64_t SEGMENT_INTERVAL_ = 60 * 1e9L;                  // 60s
  static const uint64_t CHUNK_RAW_SIZE_ = 200 * 1024 * 1024;            // 200MB
  static const uint64_t SEGMENT_RAW_SIZE_ = 2ULL * 1024 * 1024 * 1024;  // 2GB
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_RECORD_HEADER_BUILDER_H_
