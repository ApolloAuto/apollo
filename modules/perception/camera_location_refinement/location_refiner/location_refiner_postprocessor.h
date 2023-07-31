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

#include <memory>
#include <string>

#include "modules/perception/camera_location_refinement/location_refiner/proto/location_refiner.pb.h"

#include "modules/perception/camera_location_refinement/interface/base_postprocessor.h"
#include "modules/perception/camera_location_refinement/location_refiner/obj_postprocessor.h"
#include "modules/perception/common/lib/interface/base_calibration_service.h"

namespace apollo {
namespace perception {
namespace camera {

class LocationRefinerPostprocessor : public BasePostprocessor {
 public:
  LocationRefinerPostprocessor();

  virtual ~LocationRefinerPostprocessor() = default;
  /**
   * @brief: set location refiner param
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(const PostprocessorInitOptions &options =
                PostprocessorInitOptions()) override;
  /**
   * @brief post refine location of 3D obstacles
   *
   * @param options options for post refine
   * @param frame
   * @return true
   * @return false
   */
  bool Process(const PostprocessorOptions &options,
               onboard::CameraFrame *frame) override;

  std::string Name() const override { return "LocationRefinerPostprocessor"; }

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
    float slope = img_w_half * algorithm::IRec(img_h - h_down - v);
    float left = img_w_half - slope * (y - v);
    float right = img_w_half + slope * (y - h_down);
    return x > left && x < right;
  }

 private:
  std::unique_ptr<ObjPostProcessor> postprocessor_;
  std::shared_ptr<BaseCalibrationService> calibration_service_;
  location_refiner::LocationRefinerParam location_refiner_param_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
