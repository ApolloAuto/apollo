/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/common/lib/registerer/registerer.h"

#include "modules/perception/common/base/object.h"
#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/common/radar/common/radar_frame.h"
#include "modules/perception/radar4d_detection/lib/detector/point_pillars_detection/point_pillars.h"
#include "modules/perception/radar4d_detection/interface/base_detector.h"
#include "modules/perception/radar4d_detection/lib/detector/point_pillars_detection/proto/model_param.pb.h"

namespace apollo {
namespace perception {
namespace radar4d {

class Radar4dDetection : public BaseDetector {
 public:
  /**
   * @brief Construct a new Point Pillars Detection object
   * 
   */
  Radar4dDetection();

  /**
   * @brief Destroy the Point Pillars Detection object
   * 
   */
  virtual ~Radar4dDetection() = default;

  /**
   * @brief Init of Point Pillars Detection object
   * 
   * @param options radar detection init options
   * @return true 
   * @return false 
   */
  bool Init(const DetectorInitOptions& options =
                DetectorInitOptions()) override;

  /**
   * @brief Detect objects using pointpillars
   * 
   * @param options 
   * @param frame 
   * @return true 
   * @return false 
   */
  bool Detect(RadarFrame* frame,
              const DetectorOptions& options) override;

  /**
   * @brief Name of PoinPillars Detector class
   * 
   * @return std::string 
   */
  std::string Name() const override { return "Radar4dDetection"; }

 private:
  void FuseCloud(
      const base::RadarPointFCloudPtr& out_cloud_ptr,
      const std::deque<base::RadarPointDCloudPtr>& fuse_clouds,
      float* out_points_array,
      const float normalizing_factor);

  void GetObjects(std::vector<std::shared_ptr<base::Object>>* objects,
                  const Eigen::Affine3d& pose, std::vector<float>& detections,
                  std::vector<int>* labels);

  void GetBoxCorner(int num_objects,
                    const std::vector<float> &detections,
                    std::vector<float> &box_corner,
                    std::vector<float> &box_rectangular);

  void GetBoxIndices(
      int num_objects,
      const std::vector<float> &detections,
      const std::vector<float> &box_corner,
      const std::vector<float> &box_rectangular,
      std::vector<std::shared_ptr<base::Object>> *objects);

  void CalObjectVelocity(
    const std::vector<std::shared_ptr<base::Object>> *objects,
    const DetectorOptions& options);

  base::ObjectSubType GetObjectSubType(int label);

  // reference pointer of radar frame
  RadarFrame* radar_frame_ref_ = nullptr;
  std::shared_ptr<base::AttributeRadarPointCloud<base::RadarPointF>>
    original_cloud_;
  std::shared_ptr<base::AttributeRadarPointCloud<base::RadarPointD>>
    original_world_cloud_;

  // PointPillars
  std::unique_ptr<PointPillars> point_pillars_ptr_;
  std::deque<base::RadarPointDCloudPtr> prev_world_clouds_;
  base::RadarPointFCloudPtr cur_cloud_ptr_;

  pointpillars::ModelParam param_;

  // point cloud range
  float x_min_;
  float x_max_;
  float y_min_;
  float y_max_;
  float z_min_;
  float z_max_;

  // time statistics
  double cloud_to_array_time_ = 0.0;
  double inference_time_ = 0.0;
  double collect_time_ = 0.0;

  DISALLOW_COPY_AND_ASSIGN(Radar4dDetection);
};  // class Radar4dDetection

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
