/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
#include <vector>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "modules/perception/common/base/object.h"
#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/common/lidar/common/lidar_frame.h"
#include "modules/perception/lidar_detection/detector/point_pillars_detection/point_pillars.h"
#include "modules/perception/lidar_detection/interface/base_lidar_detector.h"
#include "modules/perception/common/interface/base_down_sample.h"
#include "modules/perception/lidar_detection/detector/mask_pillars_detection/proto/model_param.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

class MaskPillarsDetection : public BaseLidarDetector {
 public:
  /**
   * @brief Construct a new Mask Pillars Detection object
   * 
   */
  MaskPillarsDetection();

  /**
   * @brief Destroy the Mask Pillars Detection object
   * 
   */
  virtual ~MaskPillarsDetection() = default;

  /**
   * @brief Init of Mask Pillars Detection object
   * 
   * @param options lidar detection init options
   * @return true 
   * @return false 
   */
  bool Init(const LidarDetectorInitOptions& options =
                LidarDetectorInitOptions()) override;

  /**
   * @brief Detect foregorund objects using maskpillars
   * 
   * @param options lidar detector options
   * @param frame lidar frame
   * @return true 
   * @return false 
   */
  bool Detect(const LidarDetectorOptions& options, LidarFrame* frame) override;

  /**
   * @brief Name of Mask Pillars Detection module
   * 
   * @return std::string 
   */
  std::string Name() const override { return "MaskPillarsDetection"; }

 private:
  void CloudToArray(const base::PointFCloudPtr& pc_ptr, float* out_points_array,
                    float normalizing_factor);

  void FuseCloud(const base::PointFCloudPtr& out_cloud_ptr,
                 const std::deque<base::PointDCloudPtr>& fuse_clouds);

  std::vector<int> GenerateIndices(int start_index, int size, bool shuffle);

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
  std::deque<base::PointDCloudPtr> prev_world_clouds_;
  base::PointFCloudPtr cur_cloud_ptr_;

  // MaskPillars params
  maskpillars::ModelParam param_;

  std::shared_ptr<BaseDownSample> down_sample_;

  // point cloud range
  float x_min_;
  float x_max_;
  float y_min_;
  float y_max_;
  float z_min_;
  float z_max_;

  // time statistics
  double downsample_time_ = 0.0;
  double fuse_time_ = 0.0;
  double shuffle_time_ = 0.0;
  double cloud_to_array_time_ = 0.0;
  double inference_time_ = 0.0;
  double collect_time_ = 0.0;
};  // class MaskPillarsDetection

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
