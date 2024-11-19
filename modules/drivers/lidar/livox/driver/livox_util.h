/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace lidar {

typedef enum {
  kTimestampTypeNoSync = 0,    /**< No sync signal mode. */
  kTimestampTypeGptpOrPtp = 1, /**< gPTP or PTP sync mode */
  kTimestampTypeGps = 2        /**< GPS sync mode. */
} TimestampType;

/** 8bytes stamp to uint64_t stamp */
typedef union {
  struct {
    uint32_t low;
    uint32_t high;
  } stamp_word;

  uint8_t stamp_bytes[8];
  int64_t stamp;
} LdsStamp;

uint64_t GetEthPacketTimestamp(const uint8_t& timestamp_type,
                               const uint8_t* time_stamp, const uint8_t& size,
                               const uint32_t& point_size,
                               const bool use_lidar_clock = false);

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
