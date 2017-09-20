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

#ifndef MODULES_LOCALIZATION_CAMERA_LOCALIZATION_CAMERA_H_
#define MODULES_LOCALIZATION_CAMERA_LOCALIZATION_CAMERA_H_

#include <iomanip>
#include <string>

#include "opencv2/opencv.hpp"

class SceneInterface;

namespace apollo {
namespace localization {

struct PosInfo {
  double longitude;
  double latitude;
};

class CameraLocalization {
 public:
  CameraLocalization();
  ~CameraLocalization();

  bool init(const std::string& param_path);
  bool get_ego_position(const cv::Mat& image, const PosInfo& init_pos,
                        PosInfo& res_pos);

 private:
  SceneInterface* scene_interface_;
};

}  // localization
}  // apollo

#endif  // MODULES_LOCALIZATION_CAMERA_LOCALIZATION_CAMERA_H_
