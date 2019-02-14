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

#include "modules/perception/base/blob.h"
#include "modules/perception/base/distortion_model.h"
#include "modules/perception/base/image.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"

namespace apollo {
namespace perception {
namespace camera {

class UndistortionHandler {
 public:
  UndistortionHandler() { inited_ = false; }

  ~UndistortionHandler() { Release(); }

  bool set_device(int device);
  bool Init(const std::string &sensor_name, int device);
  void InitUndistortRectifyMap(const Eigen::Matrix3f &camera_model,
                               const Eigen::Matrix<float, 5, 1> distortion,
                               const Eigen::Matrix3f &R,
                               const Eigen::Matrix3f &new_camera_model,
                               int width, int height, base::Blob<float> *d_mapx,
                               base::Blob<float> *d_mapy);
  /** @brief: Processing each image
   * @params: src_img - input image array
   *         dst_img - output image array
   */
  bool Handle(const base::Image8U &src_img, base::Image8U *dst_img);
  // @brief: Release the resources
  bool Release(void);

 private:
  base::Blob<float> d_mapx_;
  base::Blob<float> d_mapy_;

  int width_ = 0;     // image cols
  int height_ = 0;    // image rows
  int in_size_ = 0;   // size of the input image in byte
  int out_size_ = 0;  // size of the output image in byte
  int device_ = 0;    // device number for gpu
  bool inited_ = 0;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
