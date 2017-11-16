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

#include "modules/perception/onboard/common_shared_data.h"

#include "gflags/gflags.h"

namespace apollo {
namespace perception {

DEFINE_int32(
    shared_data_stale_time, 5,
    "the time threshold longer than which the data becomes stale, in second");

DEFINE_int32(stamp_enlarge_factor, 100, "timestamp enlarge factor");

}  // namespace perception
}  // namespace apollo
