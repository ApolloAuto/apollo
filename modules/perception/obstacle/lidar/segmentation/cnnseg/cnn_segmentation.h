/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_CNNSEG_CNN_SEGMENTATION_H_  // NOLINT
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_CNNSEG_CNN_SEGMENTATION_H_  // NOLINT

#include <memory>
#include <string>
#include <vector>

#include "caffe/caffe.hpp"

#include "modules/perception/obstacle/lidar/segmentation/cnnseg/proto/cnnseg.pb.h"

#include "modules/common/log.h"
#include "modules/common/time/timer.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/lidar/interface/base_segmentation.h"
#include "modules/perception/obstacle/lidar/segmentation/cnnseg/cluster2d.h"
#include "modules/perception/obstacle/lidar/segmentation/cnnseg/feature_generator.h"

namespace apollo {
namespace perception {

class CNNSegmentation : public BaseSegmentation {
 public:
  CNNSegmentation() : BaseSegmentation() {}
  ~CNNSegmentation() {}

  bool Init() override;

  bool Segment(const pcl_util::PointCloudPtr& pc_ptr,
               const pcl_util::PointIndices& valid_indices,
               const SegmentationOptions& options,
               std::vector<std::shared_ptr<Object>>* objects) override;

  std::string name() const override { return "CNNSegmentation"; }

  float range() const { return range_; }
  int width() const { return width_; }
  int height() const { return height_; }

 private:
  bool GetConfigs(std::string* config_file, std::string* proto_file,
                  std::string* weight_file);
  // range of bird-view field (for each side)
  float range_ = 0.0;
  // number of cells in bird-view width
  int width_ = 0;
  // number of cells in bird-view height
  int height_ = 0;

  // paramters of CNNSegmentation
  apollo::perception::cnnseg::CNNSegParam cnnseg_param_;
  // Caffe network object
  std::shared_ptr<caffe::Net<float>> caffe_net_;

  // bird-view raw feature generator
  std::shared_ptr<cnnseg::FeatureGenerator<float>> feature_generator_;

  // center offset prediction
  boost::shared_ptr<caffe::Blob<float>> instance_pt_blob_;
  // objectness prediction
  boost::shared_ptr<caffe::Blob<float>> category_pt_blob_;
  // fg probability prediction
  boost::shared_ptr<caffe::Blob<float>> confidence_pt_blob_;
  // object height prediction
  boost::shared_ptr<caffe::Blob<float>> height_pt_blob_;
  // raw features to be input into network
  boost::shared_ptr<caffe::Blob<float>> feature_blob_;
  // class prediction
  boost::shared_ptr<caffe::Blob<float>> class_pt_blob_;

  // use all points of cloud to compute features
  bool use_full_cloud_ = false;

  // clustering model for post-processing
  std::shared_ptr<cnnseg::Cluster2D> cluster2d_;

  // timer
  common::time::Timer timer_;

  DISALLOW_COPY_AND_ASSIGN(CNNSegmentation);
};

REGISTER_SEGMENTATION(CNNSegmentation);

}  // namespace perception
}  // namespace apollo

#endif
