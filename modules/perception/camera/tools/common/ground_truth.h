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

#include "modules/perception/camera/common/camera_frame.h"

namespace apollo {
namespace perception {
namespace camera {

/**
 * @brief Read KITTI labels and fill to frame->detected_objects
 *
 * @param frame CameraFrame
 * @param kitti_path kitti path include labels
 * @param dist_type "H-from-h" or "H-on-h"
 * @return true
 * @return false
 */
bool LoadKittiLabel(CameraFrame *frame, const std::string& kitti_path,
    const std::string& dist_type);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
