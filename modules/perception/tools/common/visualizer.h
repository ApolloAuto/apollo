/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <string>

#include "modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h"

namespace apollo {
namespace perception {
namespace camera {

/**
 * @brief Save visualization results to file_name(.jpg)
 *
 * @param frame CameraFrame
 * @param file_name Image contains visualization results
 * @return true
 * @return false
 */
bool CameraVisualization(onboard::CameraFrame* frame,
                         const std::string& file_name);

bool TfVisualization(onboard::CameraFrame* frame, const std::string& file_name);

bool LaneVisualization(onboard::CameraFrame* frame,
                       const std::string& file_name);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
