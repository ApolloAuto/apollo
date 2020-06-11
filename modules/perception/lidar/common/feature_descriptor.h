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

#include <algorithm>
#include <vector>

#include "modules/perception/base/point_cloud.h"

namespace apollo {
namespace perception {
namespace lidar {

class FeatureDescriptor {
 public:
  explicit FeatureDescriptor(base::PointFCloud* cloud) { cloud_ = cloud; }
  FeatureDescriptor() : cloud_(nullptr) {}
  ~FeatureDescriptor() = default;

  void SetCloud(base::PointFCloud* cloud) { cloud_ = cloud; }

  void ComputeHistogram(int bin_size, float* feature) {
    GetMinMaxCenter();
    int xstep = bin_size;
    int ystep = bin_size;
    int zstep = bin_size;
    int stat_len = xstep + ystep + zstep;
    std::vector<int> stat_feat(stat_len, 0);
    float xsize =
        (max_pt_.x - min_pt_.x) / static_cast<float>(xstep) + 0.000001f;
    float ysize =
        (max_pt_.y - min_pt_.y) / static_cast<float>(ystep) + 0.000001f;
    float zsize =
        (max_pt_.z - min_pt_.z) / static_cast<float>(zstep) + 0.000001f;

    int pt_num = static_cast<int>(cloud_->size());
    for (int i = 0; i < pt_num; ++i) {
      base::PointF& pt = cloud_->at(i);
      ++stat_feat[static_cast<int>((pt.x - min_pt_.x) / xsize)];
      ++stat_feat[xstep + static_cast<int>((pt.y - min_pt_.y) / ysize)];
      ++stat_feat[xstep + ystep + static_cast<int>((pt.z - min_pt_.z) / zsize)];
    }

    feature[0] = center_pt_.x / 10.0f;
    feature[1] = center_pt_.y / 10.0f;
    feature[2] = center_pt_.z;
    feature[3] = xsize;
    feature[4] = ysize;
    feature[5] = zsize;
    feature[6] = static_cast<float>(pt_num);
    for (size_t i = 0; i < stat_feat.size(); ++i) {
      feature[i + 7] =
          static_cast<float>(stat_feat[i]) / static_cast<float>(pt_num);
    }
  }

 private:
  void GetMinMaxCenter() {
    float xsum = 0.0f;
    float ysum = 0.0f;
    float zsum = 0.0f;
    min_pt_.x = min_pt_.y = min_pt_.z = FLT_MAX;
    max_pt_.x = max_pt_.y = max_pt_.z = -FLT_MAX;

    float pt_num = static_cast<float>(cloud_->size());
    for (int i = 0; i < static_cast<int>(pt_num); ++i) {
      base::PointF& pt = cloud_->at(i);
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

  base::PointFCloud* cloud_;
  base::PointF min_pt_;
  base::PointF max_pt_;
  base::PointF center_pt_;
};  // class FeatureDescriptor

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
