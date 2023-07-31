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
#include <string>
#include <vector>

#include "gtest/gtest_prod.h"

#include "modules/perception/lidar_detection/detector/cnn_segmentation/proto/model_param.pb.h"

#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/inference/inference.h"
#include "modules/perception/common/inference/inference_factory.h"
#include "modules/perception/lidar_detection/detector/cnn_segmentation/feature_generator.h"
#include "modules/perception/lidar_detection/detector/cnn_segmentation/spp_engine/spp_engine.h"
#include "modules/perception/lidar_detection/interface/base_lidar_detector.h"

namespace apollo {
namespace perception {
namespace lidar {

class CNNSegmentation : public BaseLidarDetector {
 public:
  /**
   * @brief Construct a new CNNSegmentation object
   * 
   */
  CNNSegmentation() = default;

  /**
   * @brief Destroy the CNNSegmentation object
   * 
   */
  virtual ~CNNSegmentation() = default;

  /**
   * @brief Init of CNNSegmentation object
   * 
   * @param options lidar detection init options
   * @return true 
   * @return false 
   */
  bool Init(const LidarDetectorInitOptions& options =
                LidarDetectorInitOptions()) override;

  /**
   * @brief Detect foreground objects using CNNSegmentation
   * 
   * @param options lidar detection options
   * @param frame lidar frame
   * @return true 
   * @return false 
   */
  bool Detect(const LidarDetectorOptions& options, LidarFrame* frame) override;

  /**
   * @brief Name of CNNSegmentation module
   * 
   * @return std::string 
   */
  std::string Name() const override { return "CNNSegmentation"; }

 private:
  bool InitClusterAndBackgroundSegmentation();

  void GetObjectsFromSppEngine(
      std::vector<std::shared_ptr<base::Object>>* objects);

  void MapPointToGrid(
      const std::shared_ptr<base::AttributePointCloud<base::PointF>>& pc_ptr);

 private:
  cnnseg::ModelParam model_param_;
  std::shared_ptr<inference::Inference> inference_;
  std::shared_ptr<FeatureGenerator> feature_generator_;

  // output blobs
  std::shared_ptr<base::Blob<float>> instance_pt_blob_;
  std::shared_ptr<base::Blob<float>> category_pt_blob_;
  std::shared_ptr<base::Blob<float>> confidence_pt_blob_;
  std::shared_ptr<base::Blob<float>> classify_pt_blob_;
  std::shared_ptr<base::Blob<float>> heading_pt_blob_;
  std::shared_ptr<base::Blob<float>> height_pt_blob_;
  // input blobs
  std::shared_ptr<base::Blob<float>> feature_blob_;

  // feature parameters
  float range_ = 0.f;
  int width_ = 0;
  int height_ = 0;
  float min_height_ = 0.f;
  float max_height_ = 0.f;

  // 1-d index in feature map of each point
  std::vector<int> point2grid_;

  // spp engine
  SppEngine spp_engine_;

  // reference pointer of lidar frame
  LidarFrame* lidar_frame_ref_ = nullptr;
  std::shared_ptr<base::AttributePointCloud<base::PointF>> original_cloud_;
  std::shared_ptr<base::AttributePointCloud<base::PointD>>
      original_world_cloud_;
  std::shared_ptr<base::AttributePointCloud<base::PointF>> roi_cloud_;
  std::shared_ptr<base::AttributePointCloud<base::PointD>> roi_world_cloud_;
  int gpu_id_ = -1;

  // time statistics
  double mapping_time_ = 0.0;
  double feature_time_ = 0.0;
  double infer_time_ = 0.0;
  double fg_seg_time_ = 0.0;
  double collect_time_ = 0.0;

  // sensor_name
  std::string sensor_name_;

 private:
  const int kDefaultPointCloudSize = 120000;

  FRIEND_TEST(CNNSegmentationTest, cnn_segmentation_sequence_test);
  FRIEND_TEST(CNNSegmentationTest, cnn_segmentation_test);
};  // class CNNSegmentation

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
