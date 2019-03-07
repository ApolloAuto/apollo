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
#include <vector>

#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/lib/registerer/registerer.h"

#include "modules/perception/camera/lib/interface/base_init_options.h"

namespace apollo {
namespace perception {
namespace camera {

struct CalibratorInitOptions : public BaseInitOptions {
  int image_width = 0;
  int image_height = 0;
  float focal_x = 0.0f;
  float focal_y = 0.0f;
  float cx = 0.0f;
  float cy = 0.0f;
};

struct CalibratorOptions {
  std::shared_ptr<std::vector<base::LaneLine>> lane_objects;
  std::shared_ptr<Eigen::Affine3d> camera2world_pose;
  double *timestamp = nullptr;
};

class BaseCalibrator {
 public:
  BaseCalibrator() = default;

  virtual ~BaseCalibrator() = default;

  virtual bool Init(
      const CalibratorInitOptions &options = CalibratorInitOptions()) = 0;

  // @brief: refine 3D location of detected obstacles.
  // @param [in]: options
  // @param [in/out]: pitch_angle
  virtual bool Calibrate(const CalibratorOptions &options,
                         float *pitch_angle) = 0;

  virtual std::string Name() const = 0;

  BaseCalibrator(const BaseCalibrator &) = delete;
  BaseCalibrator &operator=(const BaseCalibrator &) = delete;
};  // class BaseCalibrator

PERCEPTION_REGISTER_REGISTERER(BaseCalibrator);
#define REGISTER_CALIBRATOR(name) \
  PERCEPTION_REGISTER_CLASS(BaseCalibrator, name)

}  // namespace camera
}  // namespace perception
}  // namespace apollo
