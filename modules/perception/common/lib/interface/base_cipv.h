/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <string>

#include "Eigen/Dense"
#include "Eigen/Eigen"
#include "Eigen/Geometry"

#include "cyber/common/macros.h"
#include "modules/perception/common/camera/common/camera_frame.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace camera {

struct CipvInitOptions : public BaseInitOptions {
  float min_laneline_length_for_cipv = 2.0f;
  float average_lane_width_in_meter = 3.7f;
  float max_vehicle_width_in_meter = 1.87f;
  float average_frame_rate = 0.05f;
  bool image_based_cipv = false;
  int debug_level = 0;
};

struct CipvOptions {
  float velocity = 5.0f;
  float yaw_rate = 0.0f;
  float yaw_angle = 0.0f;
};

class BaseCipv {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Member functions
 public:
  BaseCipv() = default;
  virtual ~BaseCipv() = default;

  virtual bool Init(const Eigen::Matrix3d &homography_im2car,
      const CipvInitOptions &options = CipvInitOptions()) = 0;

  virtual bool Process(CameraFrame *frame,
                    const CipvOptions &options,
                    const Eigen::Affine3d &world2camera,
                    const base::MotionBufferPtr &motion_buffer) = 0;

  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseCipv);
};  // class BaseCipv

PERCEPTION_REGISTER_REGISTERER(BaseCipv);
#define PERCEPTION_REGISTER_CIPV(name) \
  PERCEPTION_REGISTER_CLASS(BaseCipv, name)

}  // namespace camera
}  // namespace perception
}  // namespace apollo
