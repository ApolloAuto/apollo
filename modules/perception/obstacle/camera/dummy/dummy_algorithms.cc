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

#include "modules/perception/obstacle/camera/dummy/dummy_algorithms.h"

namespace apollo {
namespace perception {
namespace obstacle {

REGISTER_CAMERA_PARSER(DummyCameraParser);
REGISTER_CAMERA_DETECTOR(DummyCameraDetector);
// REGISTER_CAMERA_LIGHT_DETECTOR(DummyCameraLightDetector);
REGISTER_CAMERA_TRACKER(DummyCameraTracker);
REGISTER_CAMERA_TRANSFORMER(DummyCameraTransformer);

}  // namespace obstacle
}  // namespace perception
}  // namespace apollo
