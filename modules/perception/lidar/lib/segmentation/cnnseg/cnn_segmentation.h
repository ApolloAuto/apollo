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

#include "modules/perception/lidar/lib/segmentation/cnnseg/proto/cnnseg_param.pb.h"
#include "modules/perception/lidar/lib/segmentation/cnnseg/proto/spp_engine_config.pb.h"

#include "modules/perception/base/blob.h"
#include "modules/perception/inference/inference.h"
#include "modules/perception/inference/inference_factory.h"
#include "modules/perception/lib/thread/thread_worker.h"
#include "modules/perception/lidar/lib/interface/base_ground_detector.h"
#include "modules/perception/lidar/lib/interface/base_roi_filter.h"
#include "modules/perception/lidar/lib/interface/base_segmentation.h"
#include "modules/perception/lidar/lib/segmentation/cnnseg/feature_generator.h"
#include "modules/perception/lidar/lib/segmentation/cnnseg/spp_engine/spp_engine.h"

namespace apollo {
namespace perception {
namespace lidar {

class CNNSegmentation : public BaseSegmentation {
 public:
  CNNSegmentation() = default;
  ~CNNSegmentation() = default;

  bool Init(const SegmentationInitOptions& options =
                SegmentationInitOptions()) override;

  bool Segment(const SegmentationOptions& options, LidarFrame* frame) override;

  std::string Name() const override { return "CNNSegmentation"; }

 private:
  bool GetConfigs(std::string* param_file, std::string* proto_file,
                  std::string* weight_file, std::string* engine_file);

  bool InitClusterAndBackgroundSegmentation();

  void GetObjectsFromSppEngine(
      std::vector<std::shared_ptr<base::Object>>* objects);

  void MapPointToGrid(
      const std::shared_ptr<base::AttributePointCloud<base::PointF>>& pc_ptr);

  CNNSegParam cnnseg_param_;
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

  // ground detector for background segmentation
  std::unique_ptr<BaseGroundDetector> ground_detector_;
  // roi filter for background segmentation
  std::unique_ptr<BaseROIFilter> roi_filter_;

  // thread worker
  lib::ThreadWorker worker_;

  // spp engine
  SppEngine spp_engine_;
  SppEngineConfig spp_engine_config_;

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
  double join_time_ = 0.0;
  double fg_seg_time_ = 0.0;
  double collect_time_ = 0.0;
  double roi_filter_time_ = 0.0;
  double ground_detector_time_ = 0.0;

  // sensor_name
  std::string sensor_name_;

  // secondary segmentation to improve miss detection
  // not found by neural networks !
  std::shared_ptr<BaseSegmentation> secondary_segmentor;

 private:
  const int kDefaultPointCloudSize = 120000;

  FRIEND_TEST(CNNSegmentationTest, cnn_segmentation_sequence_test);
  FRIEND_TEST(CNNSegmentationTest, cnn_segmentation_test);
};  // class CNNSegmentation

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
