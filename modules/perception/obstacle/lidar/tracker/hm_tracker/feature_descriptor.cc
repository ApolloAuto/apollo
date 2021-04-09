/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/obstacle/lidar/tracker/hm_tracker/feature_descriptor.h"

namespace apollo {
namespace perception {

void FeatureDescriptor::ComputeHistogram(const int bin_size,
                                         std::vector<float>* feature) {
  GetMinMaxCenter();

  int xstep = bin_size;
  int ystep = bin_size;
  int zstep = bin_size;
  int stat_len = xstep + ystep + zstep;
  std::vector<int> stat_feat(stat_len, 0);
  float xsize = (max_pt_.x - min_pt_.x) / xstep + 0.000001;
  float ysize = (max_pt_.y - min_pt_.y) / ystep + 0.000001;
  float zsize = (max_pt_.z - min_pt_.z) / zstep + 0.000001;

  int pt_num = cloud_->points.size();
  for (int i = 0; i < pt_num; ++i) {
    pcl_util::Point& pt = cloud_->points[i];
    stat_feat[floor((pt.x - min_pt_.x) / xsize)]++;
    stat_feat[xstep + floor((pt.y - min_pt_.y) / ysize)]++;
    stat_feat[xstep + ystep + floor((pt.z - min_pt_.z) / zsize)]++;
  }
  // update feature
  (*feature).resize(stat_len);
  for (size_t i = 0; i < stat_feat.size(); ++i) {
    (*feature)[i] =
        static_cast<float>(stat_feat[i]) / static_cast<float>(pt_num);
  }
}

void FeatureDescriptor::GetMinMaxCenter() {
  float xsum = 0.0;
  float ysum = 0.0;
  float zsum = 0.0;
  min_pt_.x = min_pt_.y = min_pt_.z = FLT_MAX;
  max_pt_.x = max_pt_.y = max_pt_.z = -FLT_MAX;
  // min max pt
  int pt_num = cloud_->points.size();
  for (int i = 0; i < pt_num; ++i) {
    pcl_util::Point& pt = cloud_->points[i];
    xsum += pt.x;
    ysum += pt.y;
    zsum += pt.z;
    min_pt_.x = std::min(min_pt_.x, pt.x);
    max_pt_.x = std::max(max_pt_.x, pt.x);
    min_pt_.y = std::min(min_pt_.y, pt.y);
    max_pt_.y = std::max(max_pt_.y, pt.y);
    min_pt_.z = std::min(min_pt_.z, pt.z);
    max_pt_.z = std::max(max_pt_.z, pt.z);
  }
  // center position
  center_pt_.x = xsum / pt_num;
  center_pt_.y = ysum / pt_num;
  center_pt_.z = zsum / pt_num;
}

}  // namespace perception
}  // namespace apollo
