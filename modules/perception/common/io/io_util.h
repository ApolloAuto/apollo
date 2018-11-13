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
#pragma once

#include <string>
#include <vector>

#include "Eigen/Dense"
#include "google/protobuf/message.h"

#include "modules/perception/base/distortion_model.h"
#include "modules/perception/base/omnidirectional_model.h"

namespace apollo {
namespace perception {
namespace common {

bool ReadPoseFile(const std::string &filename, Eigen::Affine3d *pose,
                  int *frame_id, double *time_stamp);

bool LoadBrownCameraIntrinsic(const std::string &yaml_file,
                              base::BrownCameraDistortionModel *model);

// @brief: load ocam parameters
//         https://sites.google.com/site/scarabotix/ocamcalib-toolbox
bool LoadOmnidirectionalCameraIntrinsics(
    const std::string &yaml_file,
    base::OmnidirectionalCameraDistortionModel *model);

bool GetFileList(const std::string &path, const std::string &suffix,
                 std::vector<std::string> *files);

}  // namespace common
}  // namespace perception
}  // namespace apollo
