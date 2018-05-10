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
#include "modules/perception/traffic_light/base/tl_shared_data.h"

namespace apollo {
namespace perception {
namespace traffic_light {

const int kCountCameraId(static_cast<int>(CAMERA_ID_COUNT));
const int kLongFocusIdx(static_cast<int>(LONG_FOCUS));
const int kShortFocusIdx(static_cast<int>(SHORT_FOCUS));
std::vector<int> image_border_size(kCountCameraId, 100);

std::map<TLColor, std::string> kColorStr = {{UNKNOWN_COLOR, "unknown"},
                                            {RED, "red"},
                                            {GREEN, "green"},
                                            {YELLOW, "yellow"},
                                            {BLACK, "black"}};
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
