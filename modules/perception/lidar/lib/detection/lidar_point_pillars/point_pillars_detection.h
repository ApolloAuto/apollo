/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/perception/base/object.h"
#include "modules/perception/base/point_cloud.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/lidar/lib/detection/lidar_point_pillars/point_pillars.h"

namespace apollo {
namespace perception {
namespace lidar {

struct DetectionInitOptions {};

struct DetectionOptions {};

class PointPillarsDetection {
 public:
  PointPillarsDetection() = default;
  ~PointPillarsDetection() = default;

  bool Init(const DetectionInitOptions& options = DetectionInitOptions());

  bool Detect(const DetectionOptions& options, LidarFrame* frame);

  std::string Name() const { return "PointPillarsDetection"; }

 private:
  void PclToArray(const base::PointFCloudPtr& pc_ptr,
                  float* out_points_array,
                  const std::vector<int>& point_indexes,
                  float normalizing_factor);

  void FusePointCloudToArray(const base::PointFCloudPtr& pc_ptr,
                             float* out_points_array,
                             const std::vector<int>& point_indexes,
                             float normalizing_factor);

  std::vector<int> GenerateIndexes(int start_index, int size, bool shuffle);

  void GetObjects(std::vector<std::shared_ptr<base::Object>>* objects,
                  const Eigen::Affine3d& pose, std::vector<float>* detections,
                  std::vector<int>* labels);

  base::ObjectSubType GetObjectSubType(int label);

  // reference pointer of lidar frame
  LidarFrame* lidar_frame_ref_ = nullptr;
  std::shared_ptr<base::AttributePointCloud<base::PointF>> original_cloud_;
  std::shared_ptr<base::AttributePointCloud<base::PointD>>
      original_world_cloud_;

  // PointPillars
  std::unique_ptr<PointPillars> point_pillars_ptr_;
  std::deque<std::pair<base::PointDCloudPtr, double>> prev_point_clouds_;

  // time statistics
  double pcl_to_array_time_ = 0.0;
  double inference_time_ = 0.0;
  double collect_time_ = 0.0;
};  // class PointPillarsDetection

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
