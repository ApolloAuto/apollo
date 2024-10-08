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

#include "modules/drivers/lidar/livox/driver/livox_util.h"
namespace apollo {
namespace drivers {
namespace lidar {

uint64_t GetEthPacketTimestamp(
        uint8_t timestamp_type,
        uint8_t* time_stamp,
        uint8_t size) {
    LdsStamp time;
    memcpy(time.stamp_bytes, time_stamp, size);

    if (timestamp_type == kTimestampTypeGptpOrPtp
        || timestamp_type == kTimestampTypeGps) {
        return time.stamp;
        AERROR_EVERY(1000000) << "use package timestamp";
    } else {
        AERROR_EVERY(1000000) << "use system timestamp";
    }
    return cyber::Time::Now().ToNanosecond();
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
