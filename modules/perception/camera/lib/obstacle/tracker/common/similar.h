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

#include "modules/perception/camera/common/camera_frame.h"

namespace apollo {
namespace perception {
namespace camera {

class BaseSimilar {
 public:
  virtual bool Calc(CameraFrame *frame1, CameraFrame *frame2,
                    base::Blob<float> *sim) = 0;
};

class CosineSimilar : public BaseSimilar {
 public:
  CosineSimilar() = default;

  bool Calc(CameraFrame *frame1, CameraFrame *frame2,
            base::Blob<float> *sim) override;
};

class GPUSimilar : public BaseSimilar {
 public:
  bool Calc(CameraFrame *frame1, CameraFrame *frame2,
            base::Blob<float> *sim) override;
};
}  // namespace camera
}  // namespace perception
}  // namespace apollo
