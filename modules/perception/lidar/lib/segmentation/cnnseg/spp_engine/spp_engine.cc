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
#include "modules/perception/lidar/lib/segmentation/cnnseg/spp_engine/spp_engine.h"
#include <set>
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/common/lidar_timer.h"

namespace apollo {
namespace perception {
namespace lidar {

void SppEngine::Init(size_t width, size_t height, float range,
                     const SppParams& param) {
  // initialize connect component detector
  detector_2d_cc_.Init(height, width);
  detector_2d_cc_.SetData(data_.obs_prob_data_ref, data_.offset_data,
                          static_cast<float>(height) / (2.f * range),
                          data_.objectness_threshold);
  // initialize label image
  labels_2d_.Init(width, height);
  labels_2d_.InitRangeMask(range, param.confidence_range);
  // set parameters of dynamic map
  params_ = param;

  // initialize feature size
  width_ = width;
  height_ = height;
  range_ = range;
  // bind worker
  worker_.Bind([&]() {
    data_.confidence_pt_blob->cpu_data();
    data_.classify_pt_blob->cpu_data();
    data_.heading_pt_blob->cpu_data();
    data_.height_pt_blob->cpu_data();
    return true;
  });
  worker_.Start();
}

size_t SppEngine::ProcessConnectedComponentCluster(
    const base::PointFCloudConstPtr point_cloud, const CloudMask& mask) {
  Timer timer;
  data_.category_pt_blob->cpu_data();
  data_.instance_pt_blob->cpu_data();
  double sync_time1 = timer.toc(true);
  worker_.WakeUp();
  size_t num = detector_2d_cc_.Detect(&labels_2d_);
  double detect_time = timer.toc(true);
  worker_.Join();
  double sync_time2 = timer.toc(true);
  // labels_2d_.FilterClusters(data.confidence_data,
  //    data.confidence_threshold);
  // 2018.6.21 filter use category data to reserve long range objects
  // should be reverted after retrain model
  labels_2d_.FilterClusters(data_.confidence_data, data_.obs_prob_data_ref[0],
                            data_.confidence_threshold,
                            data_.objectness_threshold);
  double filter_time = timer.toc(true);
  if (data_.class_prob_data != nullptr) {
    labels_2d_.CalculateClusterClass(data_.class_prob_data, data_.class_num);
  }
  if (data_.heading_data != nullptr) {
    labels_2d_.CalculateClusterHeading(data_.heading_data);
  }
  if (data_.z_data != nullptr) {
    labels_2d_.CalculateClusterTopZ(data_.z_data);
  }
  double chz_time = timer.toc(true);
  // 2. process 2d to 3d
  // first sync between cluster list and label image,
  // and they shared the same cluster pointer
  clusters_ = labels_2d_;
#if 0
  float half_width = static_cast<float>(width_) / 2;
  float half_height = static_cast<float>(height_) / 2;
  int x = 0;
  int y = 0;
  float ratio_x = half_width / range_;
  float ratio_y = half_height / range_;
  int width = static_cast<int>(width_);
  int height = static_cast<int>(height_);

  for (size_t i = 0; i < point_cloud->size(); ++i) {
    if (mask.size() && mask[i] == 0) {
      continue;
    }
    const auto& point = point_cloud->at(i);
    //y = static_cast<int>((range_ - point.x) * ratio_x + 1.f) - 1;
    //x = static_cast<int>((range_ - point.y) * ratio_y + 1.f) - 1;
    float fx = (range_ - (0.707107 * (point.x + point.y))) * ratio_x;
    float fy = (range_ - (0.707107 * (point.x - point.y))) * ratio_y;
    x = fx < 0 ? -1.f : static_cast<int>(fx);
    y = fy < 0 ? -1.f : static_cast<int>(fy);

    if (x < 0 || x >= width || y < 0 || y >= height) {
      continue;
    }
    const uint16_t& label = labels_2d_[y][x];
    if (!label) {
      continue;
    }
    if (point.z <= labels_2d_.GetCluster(label-1)->top_z
        + data.top_z_threshold) {
      clusters_.AddPointSample(label - 1, point,
          point_cloud->points_height(i), i);
    }
  }
#else
  for (size_t i = 0; i < point_cloud->size(); ++i) {
    if (mask.size() && mask[i] == 0) {
      continue;
    }
    // out of range
    const int& id = data_.grid_indices[i];
    if (id < 0) {
      continue;
    }
    const auto& point = point_cloud->at(i);
    const uint16_t& label = labels_2d_[0][id];
    if (!label) {
      continue;
    }
    if (point.z <=
        labels_2d_.GetCluster(label - 1)->top_z + data_.top_z_threshold) {
      clusters_.AddPointSample(label - 1, point, point_cloud->points_height(i),
                               i);
    }
  }
#endif
  double mapping_time = timer.toc(true);
  // 5. remove empty clusters
  clusters_.RemoveEmptyClusters();
  double remove_time = timer.toc(true);

  LOG_INFO << "SegForeground: sync1 " << sync_time1
           << "\tdetect: " << detect_time << "\tsync2: " << sync_time2
           << "\tfilter: " << filter_time << "\tchz: " << chz_time
           << "\tmapping: " << mapping_time << "\tremove: " << remove_time;

  return clusters_.size();
}

size_t SppEngine::ProcessForegroundSegmentation(
    const base::PointFCloudConstPtr point_cloud) {
  mask_.clear();
  ProcessConnectedComponentCluster(point_cloud, mask_);
  LOG_INFO << "Foreground: " << clusters_.size() << " clusters";
  return clusters_.size();
}

size_t SppEngine::RemoveGroundPointsInForegroundCluster(
    const base::PointFCloudConstPtr full_point_cloud,
    const base::PointIndices& roi_indices,
    const base::PointIndices& roi_non_ground_indices) {
  mask_.Set(full_point_cloud->size(), 0);
  mask_.AddIndices(roi_indices);
  mask_.RemoveIndicesOfIndices(roi_indices, roi_non_ground_indices);
  mask_.Flip();
  // at this time, all ground points has mask value 0
  for (size_t i = 0; i < clusters_.size(); ++i) {
    clusters_[i]->RemovePoints(mask_);
  }
  clusters_.RemoveEmptyClusters();
  return clusters_.size();
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
