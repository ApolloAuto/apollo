/******************************************************************************
 * Copyright (c) 2014, Victor Matheus de Araujo Oliveira All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

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

#include "connected_component_gpu.h"

#include "texture.h"
#include "block_uf.h"

namespace apollo {
namespace perception {

using std::shared_ptr;
using std::unordered_set;
using std::vector;

ConnectedComponentGeneratorGPU::ConnectedComponentGeneratorGPU(int image_width,
                                                               int image_height)
    : image_width_(image_width),
      image_height_(image_height),
      width_(image_width),
      height_(image_height),
      roi_x_min_(0),
      roi_y_min_(0),
      roi_x_max_(image_width - 1),
      roi_y_max_(image_height - 1) {
  total_pix_ =
      static_cast<size_t>(image_width_) * static_cast<size_t>(image_height_);
  cudaChannelFormatDesc uchar_desc = cudaCreateChannelDesc<unsigned char>();
  cudaMallocArray(&img_array_, &uchar_desc, static_cast<size_t>(width_),
                  static_cast<size_t>(height_));
  cudaBindTextureToArray(img_tex, img_array_, uchar_desc);
  cudaMalloc(
      reinterpret_cast<void**>(&label_array_),
      static_cast<size_t>(width_) * static_cast<size_t>(height_) * sizeof(int));
  cudaError_t cuda_err = cudaGetLastError();
  if (cuda_err != cudaSuccess) {
    std::cerr
        << "failed to initialize 'img_array' and 'label_array' with CUDA: "
        << cudaGetErrorString(cuda_err) << std::endl;
  }
  labels_ = static_cast<int*>(malloc(total_pix_ * sizeof(int)));
  root_map_.reserve(total_pix_);
}

ConnectedComponentGeneratorGPU::ConnectedComponentGeneratorGPU(int image_width,
                                                               int image_height,
                                                               cv::Rect roi)
    : image_width_(image_width),
      image_height_(image_height),
      width_(roi.width),
      height_(roi.height),
      roi_x_min_(roi.x),
      roi_y_min_(roi.y),
      roi_x_max_(roi.x + roi.width - 1),
      roi_y_max_(roi.y + roi.height - 1) {
  if (roi_x_min_ < 0) {
    std::cerr << "x_min is less than 0: " << roi_x_min_ << std::endl;
  }
  if (roi_y_min_ < 0) {
    std::cerr << "y_min is less than 0: " << roi_y_min_ << std::endl;
  }
  if (roi_x_max_ >= image_width_) {
    std::cerr << "x_max is larger than image width: "
              << roi_x_max_ << "|"
              << image_width_ << std::endl;
  }
  if (roi_y_max_ >= image_height_) {
    std::cerr << "y_max is larger than image height: "
              << roi_y_max_ << "|"
              << image_height_ << std::endl;
  }
  total_pix_ = static_cast<size_t>(width_) * static_cast<size_t>(height_);
  cudaChannelFormatDesc uchar_desc = cudaCreateChannelDesc<unsigned char>();
  cudaMallocArray(&img_array_, &uchar_desc, static_cast<size_t>(width_),
                  static_cast<size_t>(height_));
  cudaBindTextureToArray(img_tex, img_array_, uchar_desc);
  cudaMalloc(
      reinterpret_cast<void**>(&label_array_),
      static_cast<size_t>(width_) * static_cast<size_t>(height_) * sizeof(int));
  cudaError_t cuda_err = cudaGetLastError();
  if (cuda_err != cudaSuccess) {
    std::cerr << "failed to initialize 'img_array' and 'label_array' with CUDA: "
              << cudaGetErrorString(cuda_err) << std::endl;
  }
  labels_ = static_cast<int*>(malloc(total_pix_ * sizeof(int)));
  root_map_.reserve(total_pix_);
}

bool ConnectedComponentGeneratorGPU::BlockUnionFind(const unsigned char* img) {
  cudaError_t cuda_err;

  if (width_ == image_width_) {
    size_t siz = static_cast<size_t>(width_) * static_cast<size_t>(height_) *
                 sizeof(unsigned char);
    cudaMemcpyToArray(img_array_, 0, 0, img, siz, cudaMemcpyHostToDevice);
  } else {
    size_t siz = static_cast<size_t>(width_) * sizeof(unsigned char);
    for (size_t i = 0; i < static_cast<size_t>(height_); ++i) {
      cudaMemcpyToArray(img_array_, 0, i, img, siz, cudaMemcpyHostToDevice);
      img += image_width_;
    }
  }

  dim3 block(UF_BLOCK_WIDTH, UF_BLOCK_HEIGHT);
  dim3 grid(
      static_cast<unsigned int>((width_ + UF_BLOCK_WIDTH - 1) / UF_BLOCK_WIDTH),
      static_cast<unsigned int>((height_ + UF_BLOCK_HEIGHT - 1) /
                                UF_BLOCK_HEIGHT));

  cuda_err = cudaGetLastError();
  if (cuda_err != cudaSuccess) {
    std::cerr << "failed to start block union find with CUDA: "
              << cudaGetErrorString(cuda_err) << std::endl;
    return false;
  }

  cudaThreadSetCacheConfig(cudaFuncCachePreferShared);

  block_uf::BlockUnionFindInternal<<<grid, block>>>(label_array_, width_,
                                                    height_);

  cudaThreadSetCacheConfig(cudaFuncCachePreferL1);

  block_uf::BlockUnionFindBoundary<<<grid, block>>>(label_array_, width_,
                                                    height_);

  block_uf::BlockUnionFindRoot<<<grid, block>>>(label_array_, width_, height_);

  cudaMemcpy(
      labels_, label_array_,
      static_cast<size_t>(width_) * static_cast<size_t>(height_) * sizeof(int),
      cudaMemcpyDeviceToHost);

  cuda_err = cudaGetLastError();
  if (cuda_err != cudaSuccess) {
    std::cerr << "failed to finish block union find with CUDA: "
              << cudaGetErrorString(cuda_err) << std::endl;
    return false;
  }

  return true;
}

bool ConnectedComponentGeneratorGPU::FindConnectedComponents(
    const cv::Mat& lane_map, vector<shared_ptr<ConnectedComponent>>* cc) {
  if (lane_map.empty()) {
    std::cerr << "The input lane map is empty." << std::endl;
    return false;
  }
  if (lane_map.type() != CV_8UC1) {
    std::cerr << "The input lane map type is not CV_8UC1." << std::endl;
    return false;
  }

  if (lane_map.cols != image_width_) {
    std::cerr << "The width of input lane map does not match." << std::endl;
    return false;
  }
  if (lane_map.rows != image_height_) {
    std::cerr << "The height of input lane map does not match." << std::endl;
    return false;
  }

  if (cc == NULL) {
    std::cerr << "The pointer of output connected components is null."
              << std::endl;
    return false;
  }

  cc->clear();

  const unsigned char* img =
      lane_map.data + roi_y_min_ * image_width_ + roi_x_min_;

  BlockUnionFind(img);

  int cur_idx = 0;
  int curt_label = 0;
  int cc_count = 0;
  root_map_.assign(total_pix_, -1);
  for (int y = roi_y_min_; y <= roi_y_max_; ++y) {
    for (int x = roi_x_min_; x <= roi_x_max_; ++x) {
      curt_label = labels_[cur_idx];

      if (curt_label >= 0) {
        if (curt_label >= static_cast<int>(total_pix_)) {
          std::cerr << "curt_label should be smaller than root_map.size() "
                    << curt_label << " (" << total_pix_ << ")." << std::endl;
          return false;
        }
        if (root_map_[curt_label] != -1) {
          cc->at(root_map_[curt_label])->AddPixel(x, y);
        } else {
          cc->push_back(std::make_shared<ConnectedComponent>(x, y));
          root_map_[curt_label] = cc_count++;
        }
      }

      ++cur_idx;
    }  // end for x
  }    // end for y

  std::cout << "#cc = " << cc_count << std::endl;

  return true;
}

}  // namespace perception
}  // namespace apollo
