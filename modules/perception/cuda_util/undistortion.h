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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_UNDISTORTION_UNDISTORTION_H
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_UNDISTORTION_UNDISTORTION_H

#include <cuda_runtime.h>
#include <stdint.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <vector>
#define CUDA_CHECK_APOLLO(condition)          \
  do {                                        \
    cudaError_t error = condition;            \
    if (error != cudaSuccess) {               \
      std::cout << cudaGetErrorString(error); \
      return static_cast<int>(error);         \
    }                                         \
  } while (0)

namespace apollo {
namespace perception {

class ImageGpuPreprocessHandler {
 public:
  ImageGpuPreprocessHandler() {
    _inited = false;
    _d_mapx = NULL;
    _d_mapy = NULL;
    _d_rgb = NULL;
    _d_dst = NULL;
  }

  ~ImageGpuPreprocessHandler() { release(); }

  inline int set_device(int dev) {
    CUDA_CHECK_APOLLO(cudaSetDevice(_dev_no));
    return 0;
  }
  int init(const std::string &intrinsics_path, int dev);
  int handle(uint8_t *src, uint8_t *dst);
  int release(void);

 private:
  int load_camera_intrinsics(const std::string &intrinsics_path, int *width,
                             int *height, std::vector<double> *D,
                             std::vector<double> *K);

  float *_d_mapx;
  float *_d_mapy;
  uint8_t *_d_rgb;
  uint8_t *_d_dst;

  int _width;     // image cols
  int _height;    // image rows
  int _in_size;   // size of the input image in byte
  int _out_size;  // size of the output image in byte
  int _dev_no;    // device number for gpu
  bool _inited;
  const int CHANNEL = 3;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_UNDISTORTION_UNDISTORTION_H
