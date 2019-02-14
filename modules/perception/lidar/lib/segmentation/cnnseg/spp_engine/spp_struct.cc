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
#include "modules/perception/lidar/lib/segmentation/cnnseg/spp_engine/spp_struct.h"

namespace apollo {
namespace perception {
namespace lidar {

void SppData::MakeReference(size_t width, size_t height, float range) {
  obs_prob_data = category_pt_blob->mutable_cpu_data();
  offset_data = instance_pt_blob->mutable_cpu_data();
  confidence_data = confidence_pt_blob->mutable_cpu_data();
  if (height_pt_blob != nullptr) {
    z_data = height_pt_blob->mutable_cpu_data();
  }
  if (classify_pt_blob != nullptr) {
    class_prob_data = classify_pt_blob->mutable_cpu_data();
  }
  if (heading_pt_blob != nullptr) {
    heading_data = heading_pt_blob->mutable_cpu_data();
  }

  if (obs_prob_data_ref == nullptr) {
    obs_prob_data_ref = new float*[height];
  }
  for (size_t i = 0; i < height; ++i) {
    obs_prob_data_ref[i] = obs_prob_data + i * width;
  }

  data_width = width;
  data_height = height;
  data_range = range;
  data_size = width * height;
}

SppData::~SppData() {
  if (obs_prob_data_ref) {
    delete[] obs_prob_data_ref;
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
