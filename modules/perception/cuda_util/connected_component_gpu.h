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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_CC_GPU_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_CC_GPU_H_

#include <cuda.h>
#include <cuda_runtime.h>
#include <opencv2/core/core.hpp>

#include <memory>
#include <vector>
#include <iostream>

#include "modules/perception/obstacle/camera/lane_post_process/common/connected_component.h"

namespace apollo {
namespace perception {

class ConnectedComponentGeneratorGPU {
 public:
  ConnectedComponentGeneratorGPU(int image_width, int image_height);
  ConnectedComponentGeneratorGPU(int image_width, int image_height,
                                 cv::Rect roi);

  ~ConnectedComponentGeneratorGPU() {
    cudaFree(label_array_);
    cudaFreeArray(img_array_);

    cudaError_t cuda_err = cudaGetLastError();
    if (cuda_err != cudaSuccess) {
      std::cerr << "failed to release label_array and img_array with CUDA: "
                << cudaGetErrorString(cuda_err) << std::endl;
    }

    free(labels_);
  }

  bool FindConnectedComponents(
      const cv::Mat& lane_map,
      std::vector<std::shared_ptr<ConnectedComponent>>* cc);

 private:
  bool BlockUnionFind(const unsigned char* img);

 private:
  size_t total_pix_;
  int image_width_;
  int image_height_;

  int width_;
  int height_;
  int roi_x_min_;
  int roi_y_min_;
  int roi_x_max_;
  int roi_y_max_;

  int* labels_ = NULL;
  cudaArray* img_array_ = NULL;
  int* label_array_ = NULL;
  std::vector<int> root_map_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_CC_GPU_H_
