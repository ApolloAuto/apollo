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
#include "modules/perception/onboard/msg_buffer/msg_buffer.h"

namespace apollo {
namespace perception {
namespace onboard {

DEFINE_int32(obs_msg_buffer_size, 200, "buffer size for odometry_subscriber");
DEFINE_double(obs_buffer_match_precision, 0.01,
              "match_precision for odometry_subscriber");

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
