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

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

#include "modules/perception/camera/lib/calibrator/laneline/lane_based_calibrator.h"
#include "modules/perception/camera/test/camera_lib_calibrator_laneline_app_util.h"

namespace apollo {
namespace perception {
namespace camera {

bool ParseOneLaneLine(const std::string &s, LaneLine *lane_line);

bool LoadLaneDet(const std::string &filename, EgoLane *ego_lane);

// bool LoadLaneDets(const std::string path, const std::string suffix,
//                   std::vector<std::string> *fnames,  // named with time-stamp
//                   std::vector<EgoLane> *ego_lanes);

std::vector<std::string> Split(const std::string &s,
                               const std::string &separator);

bool LoadCamera2WorldTfs(const std::string &filename,
                         std::vector<std::string> *frame_list,
                         std::vector<double> *time_stamps,
                         std::vector<Eigen::Matrix4d> *camera2world);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
