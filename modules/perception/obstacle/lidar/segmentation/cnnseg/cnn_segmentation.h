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

#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_CNNSEG_CNN_SEGMENTATION_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_CNNSEG_CNN_SEGMENTATION_H_

#include <memory>
#include <string>

#include "caffe/caffe.hpp"
#include "modules/common/log.h"
#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/lidar/interface/base_segmentation.h"
#include "modules/perception/obstacle/lidar/segmentation/cnnseg/proto/cnnseg.pb.h"
#include "modules/perception/obstacle/lidar/segmentation/cnnseg/util.h"
#include "modules/perception/obstacle/lidar/segmentation/cnnseg/feature_generator.h"
#include "modules/perception/obstacle/lidar/segmentation/cnnseg/cluster2d.h"

namespace apollo {
namespace perception {

class CNNSegmentation : public BaseSegmentation {
 public:
  CNNSegmentation() : BaseSegmentation() {
  }

  ~CNNSegmentation() {
  }

  virtual bool Init() override;

  virtual bool Segment(const pcl_util::PointCloudPtr& pc_ptr,
                       const pcl_util::PointIndices& valid_indices,
                       const SegmentationOptions& options,
                       std::vector<ObjectPtr>* objects) override;

  virtual std::string name() const override {
    return "CNNSegmentation";
  }

 private:
  bool GetConfigs(std::string& config_file,
                  std::string& proto_file,
                  std::string& weight_file);

  int range_;
  int width_;
  int height_;

  apollo::perception::cnnseg::CNNSegParam cnnseg_param_;
  std::shared_ptr<caffe::Net<float> > caffe_net_;

  std::shared_ptr<cnnseg::FeatureGenerator<float>> feature_generator_;

  boost::shared_ptr<caffe::Blob<float> > instance_pt_blob_;          // center offset vector
  boost::shared_ptr<caffe::Blob<float> > category_pt_blob_;          //
  boost::shared_ptr<caffe::Blob<float> > confidence_pt_blob_;        //
  boost::shared_ptr<caffe::Blob<float> > height_pt_blob_;
  boost::shared_ptr<caffe::Blob<float> > feature_blob_;

  bool use_full_cloud_;

  std::shared_ptr<cnnseg::Cluster2D> cluster2d_;

  cnnseg::Timer timer_;
  double feat_time_ = 0.0;
  double network_time_ = 0.0;
  double clust_time_ = 0.0;
  double post_time_ = 0.0;
  double tot_time_ = 0.0;

  DISALLOW_COPY_AND_ASSIGN(CNNSegmentation);
};
} // namespace perception
} // namespace apollo

#endif // MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_CNNSEG_CNN_SEGMENTATION_H_
