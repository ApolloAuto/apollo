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

#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_CNNSEG_FEATURE_GENERATOR_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_CNNSEG_FEATURE_GENERATOR_H

#include <string>
#include <cmath>
#include "caffe/caffe.hpp"
#include "modules/common/log.h"
#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/perception/obstacle/lidar/segmentation/cnnseg/proto/cnnseg.pb.h"

namespace apollo {
namespace perception {
namespace cnnseg {

template <typename Dtype>
class FeatureGenerator {
 public:
  FeatureGenerator() {}

  ~FeatureGenerator() {
//#ifndef CPU_ONLY
//    release_gpu_mem();
//#endif
  }

  bool Init(const FeatureParam& feature_param, caffe::Blob<Dtype>* out_blob);

  inline void Generate(const apollo::perception::pcl_util::PointCloudConstPtr& pc_ptr) {
//#ifdef CPU_ONLY
//    AINFO << "Generate raw features with CPU for CNNSegmentation.";
    GenerateByCpu(pc_ptr);
//#else
//    AINFO << "Generate raw features with GPU for CNNSegmentation.";
//    GenerateByGpu(pc_ptr);
//#endif
  }

//#ifdef CPU_ONLY
  void GenerateByCpu(const apollo::perception::pcl_util::PointCloudConstPtr& pc_ptr);
//#else
//  void GenerateByGpu(const apollo::perception::pcl_util::PointCloudConstPtr& pc_ptr);
//#endif

  inline std::string name() const { return "FeatureGenerator"; }

 private:
  Dtype LogCount(int count) {
    if (count < static_cast<int>(log_table_.size())) {
      return log_table_[count];
    }
    return std::log(static_cast<Dtype>(1 + count));
  }

  std::vector<Dtype> log_table_;

  int width_;
  int height_;
  int data_channel_;
  int range_;

  float min_height_;
  float max_height_;

  // raw feature data
  Dtype* max_height_data_  = nullptr;
  Dtype* mean_height_data_  = nullptr;
  Dtype* count_data_  = nullptr;
  Dtype* direction_data_  = nullptr;
  Dtype* top_intensity_data_  = nullptr;
  Dtype* mean_intensity_data_  = nullptr;
  Dtype* distance_data_  = nullptr;
  Dtype* nonempty_data_  = nullptr;

  // point index in feature map
  std::vector<int> map_idx_;

  // output Caffe blob
  caffe::Blob<Dtype>* out_blob_ = nullptr;

//#ifndef CPU_ONLY
//  apollo::perception::pcl_util::Point *pc_gpu_ = nullptr;
//  int pc_gpu_size_ = 0;
//  std::shared_ptr<caffe::Blob<Dtype>> log_table_blob_;
//#endif
};

typedef FeatureGenerator<float> FP32FeatureGenerator;
typedef FeatureGenerator<double> FP64FeatureGenerator;

}  // namespace cnnseg
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_CNNSEG_FEATURE_GENERATOR_H
