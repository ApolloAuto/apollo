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
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest_prod.h"

#include "modules/perception/lidar/lib/segmentation/cnnseg/proto/cnnseg_param.pb.h"

#include "modules/perception/base/blob.h"
#include "modules/perception/base/point_cloud.h"

namespace apollo {
namespace perception {
namespace lidar {

class FeatureGenerator {
 public:
  FeatureGenerator() {
#ifndef PERCEPTION_CPU_ONLY
    pc_gpu_size_ = kMaxPointCloudGPUSize;
    BASE_CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&pc_gpu_),
                               pc_gpu_size_ * sizeof(base::PointF)));
    BASE_CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&point2grid_gpu_),
                               pc_gpu_size_ * sizeof(int)));
#endif
  }

  virtual ~FeatureGenerator() {
#ifndef PERCEPTION_CPU_ONLY
    ReleaseGPUMemory();
#endif
  }

  bool Init(const FeatureParam& feature_param, base::Blob<float>* out_blob);

  void Generate(const base::PointFCloudPtr& pc_ptr,
                const std::vector<int>& point2grid) {
#ifndef PERCEPTION_CPU_ONLY
    GenerateGPU(pc_ptr, point2grid);
#else
    GenerateCPU(pc_ptr, point2grid);
#endif
  }

  inline std::string Name() const { return "FeatureGenerator"; }

 private:
#ifndef PERCEPTION_CPU_ONLY
  void GenerateGPU(const base::PointFCloudPtr& pc_ptr,
                   const std::vector<int>& point2grid);
  void ReleaseGPUMemory();
#endif
  void GenerateCPU(const base::PointFCloudPtr& pc_ptr,
                   const std::vector<int>& point2grid);

  float LogCount(int count) {
    if (count < static_cast<int>(log_table_.size())) {
      return log_table_[count];
    }
    return logf(static_cast<float>(1 + count));
  }

  // log table for CPU, with std::vector type
  std::vector<float> log_table_;
  // log table for GPU, with base::Blob type
  std::shared_ptr<base::Blob<float>> log_blob_;
  const int kMaxLogNum = 256;

  // feature param
  int width_ = 0;
  int height_ = 0;
  float range_ = 0.0f;
  float min_height_ = 0.0f;
  float max_height_ = 0.0f;
  bool use_intensity_feature_ = false;
  bool use_constant_feature_ = false;

  // raw feature data
  float* max_height_data_ = nullptr;
  float* mean_height_data_ = nullptr;
  float* count_data_ = nullptr;
  float* direction_data_ = nullptr;
  float* top_intensity_data_ = nullptr;
  float* mean_intensity_data_ = nullptr;
  float* distance_data_ = nullptr;
  float* nonempty_data_ = nullptr;

  // 1-d index in feature map of each point
  std::vector<int> map_idx_;

  // output feature blob
  base::Blob<float>* out_blob_ = nullptr;

  // param for managing gpu memory, only for cuda code
  base::PointF* pc_gpu_ = nullptr;
  int* point2grid_gpu_ = nullptr;
  int pc_gpu_size_ = 0;
  const int kMaxPointCloudGPUSize = 120000;
  const int kGPUThreadSize = 512;

  // for TEST only
  FRIEND_TEST(FeatureGeneratorTest, basic_test);
  friend class FeatureGeneratorTest;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
