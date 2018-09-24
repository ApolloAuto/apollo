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
#ifndef PERCEPTION_LIDAR_LIB_SEGMENTATION_CNNSEG_H_
#define PERCEPTION_LIDAR_LIB_SEGMENTATION_CNNSEG_H_

#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>

#include "modules/perception/base/blob.h"
#include "modules/perception/inference/inference.h"
#include "modules/perception/lib/thread/thread_worker.h"
#include "modules/perception/lidar/lib/interface/base_ground_detector.h"
#include "modules/perception/lidar/lib/interface/base_roi_filter.h"
#include "modules/perception/lidar/lib/interface/base_segmentation.h"
#include "modules/perception/lidar/lib/segmentation/cnnseg/feature_generator.h"
#include "modules/perception/lidar/lib/segmentation/cnnseg/proto/cnnseg_param.pb.h"
#include "modules/perception/lidar/lib/segmentation/cnnseg/proto/spp_engine_config.pb.h"
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

  void GetObjectsFromSppEngine(std::vector<base::ObjectPtr>* objects);

  void MapPointToGrid(const base::PointFCloudPtr& pc_ptr);

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
  base::PointFCloudPtr original_cloud_;
  base::PointDCloudPtr original_world_cloud_;
  base::PointFCloudPtr roi_cloud_;
  base::PointDCloudPtr roi_world_cloud_;
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

 private:
  const int kDefaultPointCloudSize = 120000;
};  // class CNNSegmentation

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_LIDAR_LIB_SEGMENTATION_CNNSEG_H_
