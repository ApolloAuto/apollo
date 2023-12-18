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

#include "modules/perception/common/base/blob.h"

namespace apollo {
namespace perception {
namespace lidar {

struct SppData {
  float* obs_prob_data = nullptr;
  float* offset_data = nullptr;
  float* confidence_data = nullptr;
  float* z_data = nullptr;
  float* class_prob_data = nullptr;
  float* heading_data = nullptr;

  base::Blob<float>* instance_pt_blob = nullptr;
  base::Blob<float>* category_pt_blob = nullptr;
  base::Blob<float>* confidence_pt_blob = nullptr;
  base::Blob<float>* classify_pt_blob = nullptr;
  base::Blob<float>* heading_pt_blob = nullptr;
  base::Blob<float>* height_pt_blob = nullptr;

  float** obs_prob_data_ref = nullptr;

  int* grid_indices = nullptr;

  float objectness_threshold = 0.f;
  float confidence_threshold = 0.f;
  float top_z_threshold = 0.f;

  size_t class_num = 0;
  size_t data_width = 0;
  size_t data_height = 0;
  size_t data_size = 0;
  float data_range = 0.0f;

  void MakeReference(size_t width, size_t height, float range);

  ~SppData();
};

struct SppParams {
  float height_gap = 0.5f;
  float confidence_range = 58.f;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
