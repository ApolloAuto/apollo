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

#include <string>

#include "modules/perception/camera/lib/interface/base_obstacle_postprocessor.h"
#include "modules/perception/camera/lib/obstacle/postprocessor/location_refiner/location_refiner.pb.h"
#include "modules/perception/camera/lib/obstacle/postprocessor/location_refiner/obj_postprocessor.h"

namespace apollo {
namespace perception {
namespace camera {

class LocationRefinerObstaclePostprocessor : public BaseObstaclePostprocessor {
 public:
  LocationRefinerObstaclePostprocessor() : BaseObstaclePostprocessor() {
    postprocessor_ = new ObjPostProcessor;
  }

  virtual ~LocationRefinerObstaclePostprocessor() {
    delete postprocessor_;
    postprocessor_ = nullptr;
  }
  bool Init(const ObstaclePostprocessorInitOptions &options =
                ObstaclePostprocessorInitOptions()) override;

  // @brief: post-refine location of 3D obstacles
  // @param [in]: frame
  // @param [out]: frame
  bool Process(const ObstaclePostprocessorOptions &options,
               CameraFrame *frame) override;

  std::string Name() const override;

 private:
  bool is_in_roi(const float pt[2], float img_w, float img_h, float v,
                 float h_down) const {
    float x = pt[0];
    float y = pt[1];
    if (y < v) {
      return false;
    } else if (y > (img_h - h_down)) {
      return true;
    }
    float img_w_half = img_w / 2.0f;
    float slope = img_w_half * common::IRec(img_h - h_down - v);
    float left = img_w_half - slope * (y - v);
    float right = img_w_half + slope * (y - h_down);
    return x > left && x < right;
  }

 private:
  //  int image_width_ = 0;
  //  int image_height_ = 0;
  ObjPostProcessor *postprocessor_ = nullptr;
  location_refiner::LocationRefinerParam location_refiner_param_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
