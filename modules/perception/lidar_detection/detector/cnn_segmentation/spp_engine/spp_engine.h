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

#include <string>

#include "Eigen/Dense"

#include "modules/perception/common/lib/thread/thread_worker.h"
#include "modules/perception/common/lidar/common/cloud_mask.h"
#include "modules/perception/lidar_detection/detector/cnn_segmentation/spp_engine/spp_cluster.h"
#include "modules/perception/lidar_detection/detector/cnn_segmentation/spp_engine/spp_cluster_list.h"
#include "modules/perception/lidar_detection/detector/cnn_segmentation/spp_engine/spp_seg_cc_2d.h"
#include "modules/perception/lidar_detection/detector/cnn_segmentation/spp_engine/spp_struct.h"

namespace apollo {
namespace perception {
namespace lidar {

class SppEngine {
 public:
  SppEngine() = default;
  ~SppEngine() = default;
  // @brief: initialize spp engine
  // @param [in]: feature map width
  // @param [in]: feature map height
  // @param [in]: feature map range
  // @param [in]: sensor name
  void Init(size_t width, size_t height, float range,
            const SppParams& param = SppParams(),
            const std::string& sensor_name = "velodyne64");
  // @brief: process foreground segmentation
  // @param [in]: point cloud
  // @return: size of foreground clusters
  size_t ProcessForegroundSegmentation(
      const base::PointFCloudConstPtr point_cloud);
  // @brief: remove ground points in foreground cluster
  // @param [in]: point cloud
  // @param [in]: roi indices of point cloud
  // @param [in]: non ground indices in roi of point cloud
  size_t RemoveGroundPointsInForegroundCluster(
      const base::PointFCloudConstPtr full_point_cloud,
      const base::PointIndices& roi_indices,
      const base::PointIndices& roi_non_ground_indices);
  // @brief: get cluster list, const version
  // @return: cluster list
  inline const SppClusterList& clusters() const { return clusters_; }
  // @brief: get cluster list, const version
  // @return: cluster list
  inline SppClusterList& clusters() { return clusters_; }
  // @brief: get feature data, need to filled first
  inline SppData& GetSppData() { return data_; }
  // @brief: get feature data, need to filled first, const version
  inline const SppData& GetSppData() const { return data_; }

 private:
  // @brief: process clustering on input feature map
  // @param [in]: point cloud
  // @param [in]: point cloud mask
  size_t ProcessConnectedComponentCluster(
      const base::PointFCloudConstPtr point_cloud, const CloudMask& mask);

 private:
  // feature size
  size_t width_ = 0;
  size_t height_ = 0;
  float range_ = 0.f;
  // label image
  SppLabelImage labels_2d_;
  // clusters
  SppClusterList clusters_;
  // cloud mask
  CloudMask mask_;
  CloudMask roi_mask_;
  // clustering method
  SppCCDetector detector_2d_cc_;
  // global parameters
  SppParams params_;
  // const spp data pointer
  SppData data_;
  // thread worker for sync data
  lib::ThreadWorker worker_;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
