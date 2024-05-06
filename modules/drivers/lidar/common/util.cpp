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

#include "modules/drivers/lidar/common/util.h"

namespace apollo {
namespace drivers {
namespace lidar {

uint64_t GetNanosecondTimestampFromSecondTimestamp(double second_timestamp) {
    auto ll_i = static_cast<uint64_t>(second_timestamp);
    uint64_t ll_f = (second_timestamp - ll_i) * 1e9;
    return ll_i * 1000000000LL + ll_f;
}

double GetSecondTimestampFromNanosecondTimestamp(
        uint64_t nanosecond_timestamp) {
    uint64_t ll_i = nanosecond_timestamp / 1000000000ULL;
    uint64_t ll_f = nanosecond_timestamp - ll_i * 1000000000ULL;
    double d_f = ll_f * 1e-9;
    return static_cast<double>(ll_i) + d_f;
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
