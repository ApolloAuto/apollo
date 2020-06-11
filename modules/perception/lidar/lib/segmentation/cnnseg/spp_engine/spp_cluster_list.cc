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
#include "modules/perception/lidar/lib/segmentation/cnnseg/spp_engine/spp_cluster_list.h"

#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/lib/segmentation/cnnseg/spp_engine/spp_pool_types.h"

namespace apollo {
namespace perception {
namespace lidar {

void SppClusterList::Init(size_t size, const std::string& sensor_name) {
  sensor_name_ = sensor_name;
  clusters_.clear();
  SppClusterPool::Instance(sensor_name_).BatchGet(size, &clusters_);
}

void SppClusterList::resize(size_t size) {
  if (clusters_.size() < size) {
    SppClusterPool::Instance(sensor_name_)
        .BatchGet(size - clusters_.size(), &clusters_);
  } else {
    clusters_.resize(size);
  }
}

void SppClusterList::AddPointSample(size_t cluster_id,
                                    const base::PointF& point, float height,
                                    uint32_t point_id) {
  if (clusters_.size() <= cluster_id) {
    resize(cluster_id + 1);
  }
  clusters_[cluster_id]->AddPointSample(point, height, point_id);
}

void SppClusterList::Merge(SppClusterList* rhs) {
  clusters_.reserve(clusters_.size() + rhs->clusters_.size());
  for (size_t i = 0; i < rhs->size(); ++i) {
    clusters_.push_back((*rhs)[static_cast<int>(i)]);
  }
}

size_t SppClusterList::HeightCut(float max_gap, size_t start_id) {
  size_t size = clusters_.size();
  size_t count = 0;
  for (size_t i = start_id; i < size; ++i) {
    if (clusters_[i]->points.size() > 0) {
      if (ComputeHeightAndSplitCluster(i, max_gap)) {
        ++count;
      }
    }
  }
  AINFO << "Split " << count << " clusters in 3d";
  return count;
}

bool SppClusterList::ComputeHeightAndSplitCluster(size_t id, float max_gap) {
  if (id >= clusters_.size()) {
    return false;
  }

  clusters_[id]->SortPoint();
  std::vector<SppPoint>& points = clusters_[id]->points;
  std::vector<uint32_t>& indices = clusters_[id]->point_ids;

  float gap = 0.f;
  std::vector<size_t> split_indices(1, 0);
  for (size_t i = 1; i < points.size(); ++i) {
    if (points[i].h < 0) {
      continue;
    }
    gap = points[i].z - points[i - 1].z;
    if (gap > max_gap) {
      split_indices.push_back(i);
    }
  }
  size_t split_num = split_indices.size();
  if (split_num > 0) {
    size_t max_index = split_indices.size() - 1;
    size_t length = points.size() - split_indices.back();
    size_t max_length = length;
    for (size_t i = 1; i < split_indices.size(); ++i) {
      length = split_indices[i] - split_indices[i - 1];
      if (length > max_length) {
        max_length = length;
        max_index = i - 1;
      }
    }
    // the highest split is not stable
    if (max_index != split_indices.size() - 1) {
      size_t split_index = split_indices[max_index + 1];
      points.resize(split_index);
      indices.resize(split_index);
      return true;
    }
  }
  return false;
}

void SppClusterList::RemoveEmptyClusters() {
  size_t current = 0;
  for (size_t i = 0; i < clusters_.size(); ++i) {
    if (clusters_[i]->points.size() > 0) {
      if (current != i) {
        clusters_[current] = clusters_[i];
      }
      ++current;
    }
  }
  clusters_.resize(current);
}

void SppClusterList::EraseCluster(size_t id) {
  if (clusters_.size() <= id) {
    return;
  }
  clusters_[id] = clusters_.back();
  clusters_.resize(clusters_.size() - 1);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
