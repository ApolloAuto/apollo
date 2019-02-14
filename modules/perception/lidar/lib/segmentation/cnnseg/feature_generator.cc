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
#include "modules/perception/lidar/lib/segmentation/cnnseg/feature_generator.h"

#include "modules/perception/base/common.h"
#include "modules/perception/lidar/lib/segmentation/cnnseg/util.h"

namespace apollo {
namespace perception {
namespace lidar {

bool FeatureGenerator::Init(const FeatureParam& feature_param,
                            base::Blob<float>* out_blob) {
  // set output feature blob
  out_blob_ = out_blob;

  // set feature parameters
  range_ = feature_param.point_cloud_range();
  width_ = feature_param.width();
  height_ = feature_param.height();
  min_height_ = feature_param.min_height();
  max_height_ = feature_param.max_height();
  CHECK_EQ(width_, height_)
      << "Current implementation version requires input_width == input_height.";
  use_intensity_feature_ = feature_param.use_intensity_feature();
  use_constant_feature_ = feature_param.use_constant_feature();

  // set log lookup table
  log_table_.resize(kMaxLogNum);
  for (size_t i = 0; i < log_table_.size(); ++i) {
    log_table_[i] = std::log(static_cast<float>(1 + i));
  }

  // set output feature blob data
  float* out_blob_data = nullptr;
#ifndef PERCEPTION_CPU_ONLY
  log_blob_.reset(
      new base::Blob<float>(1, 1, 1, static_cast<int>(log_table_.size())));
  float* log_table = log_blob_->mutable_gpu_data();
  cudaMemcpy(log_table, log_table_.data(), log_table_.size() * sizeof(float),
             cudaMemcpyHostToDevice);
  out_blob_data = out_blob_->mutable_gpu_data();
#else
  out_blob_data = out_blob_->mutable_cpu_data();
#endif

  // set raw feature data
  int channel_index = 0;
  max_height_data_ = out_blob_data + out_blob_->offset(0, channel_index++);
  mean_height_data_ = out_blob_data + out_blob_->offset(0, channel_index++);
  count_data_ = out_blob_data + out_blob_->offset(0, channel_index++);
  if (use_constant_feature_) {
    direction_data_ = out_blob_data + out_blob_->offset(0, channel_index++);
  }
  if (use_intensity_feature_) {
    top_intensity_data_ = out_blob_data + out_blob_->offset(0, channel_index++);
    mean_intensity_data_ =
        out_blob_data + out_blob_->offset(0, channel_index++);
  }
  if (use_constant_feature_) {
    distance_data_ = out_blob_data + out_blob_->offset(0, channel_index++);
  }
  nonempty_data_ = out_blob_data + out_blob_->offset(0, channel_index++);
  CHECK_EQ(out_blob_->offset(0, channel_index), out_blob_->count());

  // compute direction and distance features
  if (use_constant_feature_) {
    int map_size = height_ * width_;
    std::vector<float> direction_data(map_size);
    std::vector<float> distance_data(map_size);
    for (int row = 0; row < height_; ++row) {
      for (int col = 0; col < width_; ++col) {
        int idx = row * width_ + col;
        // * row <-> x, column <-> y
        float center_x = Pixel2Pc(row, static_cast<float>(height_), range_);
        float center_y = Pixel2Pc(col, static_cast<float>(width_), range_);
        direction_data[idx] =
            static_cast<float>(std::atan2(center_y, center_x) / (2.0 * kPI));
        distance_data[idx] =
            static_cast<float>(std::hypot(center_x, center_y) / 60.0 - 0.5);
      }
    }

// memory copy direction and distance features
#ifndef PERCEPTION_CPU_ONLY
    cudaMemcpy(direction_data_, direction_data.data(),
               direction_data.size() * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(distance_data_, distance_data.data(),
               distance_data.size() * sizeof(float), cudaMemcpyHostToDevice);
#else
    memcpy(direction_data_, direction_data.data(),
           direction_data.size() * sizeof(float));
    memcpy(distance_data_, distance_data.data(),
           distance_data.size() * sizeof(float));
#endif
  }
  return true;
}

void FeatureGenerator::GenerateCPU(const base::PointFCloudPtr& pc_ptr,
                                   const std::vector<int>& point2grid) {
  // DO NOT remove this line!!!
  // Otherwise, the gpu_data will not be updated for the later frames.
  // It marks the head at cpu for blob.
  out_blob_->mutable_cpu_data();

  // fill initial value for feature blob
  const int map_size = height_ * width_;
  for (int i = 0; i < map_size; ++i) {
    max_height_data_[i] = -5.f;
  }
  memset(mean_height_data_, 0, map_size * sizeof(float));
  memset(count_data_, 0, map_size * sizeof(float));
  memset(nonempty_data_, 0, map_size * sizeof(float));
  if (use_intensity_feature_) {
    memset(top_intensity_data_, 0, map_size * sizeof(float));
    memset(mean_intensity_data_, 0, map_size * sizeof(float));
  }

  // compute features
  for (size_t i = 0; i < pc_ptr->size(); ++i) {
    int idx = point2grid[i];
    if (idx == -1) {
      continue;
    }
    const auto& pt = pc_ptr->at(i);
    float pz = pt.z;
    float pi = pt.intensity / 255.0f;
    if (max_height_data_[idx] < pz) {
      max_height_data_[idx] = pz;
      if (use_intensity_feature_) {
        top_intensity_data_[idx] = pi;
      }
    }
    mean_height_data_[idx] += static_cast<float>(pz);
    if (use_intensity_feature_) {
      mean_intensity_data_[idx] += static_cast<float>(pi);
    }
    count_data_[idx] += 1.f;
  }

  for (int i = 0; i < map_size; ++i) {
    if (count_data_[i] <= FLT_EPSILON) {
      max_height_data_[i] = 0.f;
    } else {
      mean_height_data_[i] /= count_data_[i];
      if (use_intensity_feature_) {
        mean_intensity_data_[i] /= count_data_[i];
      }
      nonempty_data_[i] = 1.f;
    }
    count_data_[i] = LogCount(static_cast<int>(count_data_[i]));
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
