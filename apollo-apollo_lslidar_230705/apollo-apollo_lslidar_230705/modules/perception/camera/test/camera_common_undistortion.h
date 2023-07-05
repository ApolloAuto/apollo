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
#include <vector>

#include "modules/perception/base/common.h"

namespace apollo {
namespace perception {
namespace camera {

class ImageGpuPreprocessHandler {
 public:
  ImageGpuPreprocessHandler() {}

  ~ImageGpuPreprocessHandler() { release(); }

  inline int set_device(int dev) {
    BASE_GPU_CHECK(cudaSetDevice(dev));
    return 0;
  }
  int init(const std::string &intrinsics_path, int dev);
  int handle(uint8_t *src, uint8_t *dst);
  int release(void);

 private:
  int load_camera_intrinsics(const std::string &intrinsics_path, int *width,
                             int *height, std::vector<double> *D,
                             std::vector<double> *K);

  float *_d_mapx = nullptr;
  float *_d_mapy = nullptr;
  uint8_t *_d_rgb = nullptr;
  uint8_t *_d_dst = nullptr;

  int _width = 0;     // image cols
  int _height = 0;    // image rows
  int _in_size = 0;   // size of the input image in byte
  int _out_size = 0;  // size of the output image in byte
  int _dev_no = 0;    // device number for gpu
  bool _inited = false;
  const int CHANNEL = 3;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
