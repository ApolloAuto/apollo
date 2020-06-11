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
#include <utility>
#include <vector>

#include "cyber/common/log.h"

#include "modules/perception/base/point_cloud.h"
#include "modules/perception/common/geometry/basic.h"

namespace apollo {
namespace perception {
namespace common {

// @brief: a filter of circular.
// if point's euclidean dist smaller than neighbour_dist with center_pt and
// bigger than or equal radius with last_pt, it will be keep.
template <typename PointT>
void DownsamplingCircular(
    const PointT& center_pt, float radius, float neighbour_dist,
    typename std::shared_ptr<const base::PointCloud<PointT>> cloud,
    typename std::shared_ptr<base::PointCloud<PointT>> down_cloud) {
  for (size_t c = 0; c < cloud->width(); ++c) {
    for (size_t r = 0; r < cloud->height(); ++r) {
      PointT tmp_pt;
      if (cloud->height() > 1) {
        const PointT* tmp_pt_ptr = cloud->at(c, r);
        if (tmp_pt_ptr == nullptr) {
          continue;
        }
        tmp_pt = *(tmp_pt_ptr);
      } else {
        tmp_pt = cloud->at(c);
      }
      if (CalculateEuclidenDist<PointT>(tmp_pt, center_pt) < radius) {
        if (down_cloud->size() == 0) {
          down_cloud->push_back(tmp_pt);
        } else {
          if (CalculateEuclidenDist<PointT>(
                  tmp_pt, down_cloud->at(down_cloud->size() - 1)) >=
              neighbour_dist) {
            down_cloud->push_back(tmp_pt);
          }
        }
      }
    }
  }
}

// @brief: like function DownsamplingCircular. Its center is origin, and have
//         a new parameter smp_ratio to downsampling.
//         if smp_ratio=1, func don't downsample, only filtering.
//         usually set velodyne_model to 64.
template <typename PointT>
void DownsamplingCircularOrgAll(
    const PointT& center_pt, int smp_ratio, float radius, int velodyne_model,
    typename std::shared_ptr<const base::PointCloud<PointT>> cloud,
    typename std::shared_ptr<base::PointCloud<PointT>> down_cloud) {
  int smp_step = smp_ratio * velodyne_model;
  down_cloud->resize(cloud->size() / smp_ratio + 1);
  size_t ii = 0;
  for (size_t ori_ii = 0; ori_ii < cloud->size(); ori_ii += smp_step) {
    for (size_t jj = ori_ii;
         jj < cloud->size() && (jj - ori_ii) < velodyne_model; ++jj) {
      const PointT& p = cloud->at(jj);
      if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
        continue;
      }
      float r = CalculateEuclidenDist2DXY<PointT>(center_pt, p);
      if (r > radius) {
        continue;
      }
      down_cloud->at(jj).x = cloud->at(jj).x;
      down_cloud->at(jj).y = cloud->at(jj).y;
      down_cloud->at(jj).z = cloud->at(jj).z;
      ++ii;
    }
  }
  down_cloud->resize(ii);
}

// @brief: like function DownsamplingCircularOrgAll, this function is
//         downsampling 2d PointCloud alternately.
//         usually set velodyne_model to 64.
template <typename PointT>
void DownsamplingCircularOrgPartial(
    const PointT& center_pt, int org_num, int smp_ratio, float radius,
    int velodyne_model,
    const typename std::shared_ptr<const base::PointCloud<PointT>> cloud,
    typename std::shared_ptr<base::PointCloud<PointT>> down_cloud,
    std::vector<std::pair<int, int>>* all_org_idx_ptr) {
  int smp_height = static_cast<int>(cloud->height()) / smp_ratio;
  int smp_width = org_num;
  if (smp_width < 1 || smp_width >= velodyne_model) {
    AERROR << "org_num error!";
    return;
  }
  size_t ii = 0;
  down_cloud->resize(smp_height * smp_width);
  all_org_idx_ptr->resize(smp_height * smp_width);
  for (size_t hh = 0; hh < smp_height; ++hh) {
    for (size_t ww = 0; ww < smp_width; ++ww) {
      int ori_hh = static_cast<int>(hh) * smp_ratio;
      const PointT* p_ptr = cloud->at(ww, ori_hh);
      if (p_ptr == nullptr) {
        continue;
      }
      const PointT& p = *(p_ptr);
      if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
        continue;
      }
      float r = CalculateEuclidenDist2DXY<PointT>(center_pt, p);
      if (r > radius) {
        continue;
      } else {
        down_cloud->at(ii) = p;
        all_org_idx_ptr->at(ii).first = static_cast<int>(hh);
        all_org_idx_ptr->at(ii).second = static_cast<int>(ww);
        ++ii;
      }
    }
  }
  down_cloud->resize(ii);
  all_org_idx_ptr->resize(ii);
}

// @brief: This function is downsampling 2d PointCloud alternately.
//         usually set velodyne_model to 64.
template <typename PointT>
void DownsamplingRectangleOrgPartial(
    int org_num, int smp_ratio, float front_range, float side_range,
    int velodyne_model,
    typename std::shared_ptr<const base::PointCloud<PointT>> cloud,
    typename std::shared_ptr<base::PointCloud<PointT>> down_cloud,
    std::vector<std::pair<int, int>>* all_org_idx_ptr) {
  int smp_height = static_cast<int>(cloud->height()) / smp_ratio;
  int smp_width = org_num;
  if (smp_width < 1 || smp_width >= velodyne_model) {
    AERROR << "org_num error!";
    return;
  }
  size_t ii = 0;
  down_cloud->resize(smp_height * smp_width);
  all_org_idx_ptr->resize(smp_height * smp_width);
  for (size_t hh = 0; hh < smp_height; ++hh) {
    for (size_t ww = 0; ww < smp_width; ++ww) {
      int ori_hh = static_cast<int>(hh) * smp_ratio;
      const PointT* p_ptr = cloud->at(ww, ori_hh);
      if (p_ptr == nullptr) {
        continue;
      }
      const PointT& p = *(p_ptr);
      if (std::isnan(p.x) || std::isnan((p.y) || std::isnan(p.z))) {
        continue;
      }
      if (fabs(p.x) > front_range || fabs(p.y) > side_range) {
        continue;
      } else {
        down_cloud->at(ii) = p;
        all_org_idx_ptr->at(ii).first = static_cast<int>(hh);
        all_org_idx_ptr->at(ii).second = static_cast<int>(ww);
        ++ii;
      }
    }
  }
  down_cloud->resize(ii);
  all_org_idx_ptr->resize(ii);
}

// @brief: a filter of neighbour rectangle.
//         usually set velodyne_model to 64.
template <typename PointT>
void DownsamplingRectangleNeighbour(
    float front_range, float side_range, double max_nei, int velo_model,
    typename std::shared_ptr<const base::PointCloud<PointT>> cloud,
    typename std::shared_ptr<base::PointCloud<PointT>> down_cloud) {
  if (cloud->width() != velo_model) {
    AERROR << "cloud->width (" << cloud->width() << ") does not match "
           << "velo_model (" << velo_model << ")";
    return;
  }
  down_cloud->resize(cloud->size());
  size_t pt_num = 0;
  for (int ww = 0; ww < cloud->width(); ++ww) {
    PointT nei_pt;
    nei_pt.x = 0;
    nei_pt.y = 0;
    nei_pt.z = 0;
    for (int hh = 0; hh < cloud->height(); ++hh) {
      const PointT* p_ptr = cloud->at(ww, hh);
      if (p_ptr == nullptr) {
        continue;
      }
      const PointT& p = *(p_ptr);
      if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
        continue;
      }
      if (fabs(p.x) > front_range || fabs(p.y) > side_range) {
        continue;
      }
      if (fabs(p.x - nei_pt.x) > max_nei || fabs(p.y - nei_pt.y) > max_nei) {
        nei_pt = p;
        down_cloud->at(pt_num++) = p;
      }
    }
  }
  down_cloud->resize(pt_num);
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
