/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <fstream>
#include <string>
#include <vector>

#include "modules/perception/common/base/object.h"
#include "modules/perception/common/camera/common/camera_frame.h"

namespace apollo {
namespace perception {
namespace camera {
int WriteLanelines(const bool enable, const std::string &save_path,
                   const std::vector<base::LaneLine> &lane_objects);

int WriteCalibrationOutput(bool enable, const std::string &out_path,
                           const CameraFrame *frame);
}  // namespace camera
}  // namespace perception
}  // namespace apollo
