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

#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_CNNSEG_FEATURE_GENERATOR_H_  // NOLINT
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_CNNSEG_FEATURE_GENERATOR_H_  // NOLINT

#include <cmath>
#include <string>
#include <vector>
#include "caffe/caffe.hpp"
#include "modules/common/log.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/obstacle/lidar/segmentation/cnnseg/proto/cnnseg.pb.h"

namespace apollo {
namespace perception {
namespace cnnseg {

template <typename Dtype>
class FeatureGenerator {
 public:
  FeatureGenerator() {}

  ~FeatureGenerator() {}

  bool Init(const FeatureParam& feature_param, caffe::Blob<Dtype>* out_blob);

  void Generate(const apollo::perception::pcl_util::PointCloudConstPtr& pc_ptr);

  inline std::string name() const { return "FeatureGenerator"; }

 private:
  Dtype LogCount(int count) {
    if (count < static_cast<int>(log_table_.size())) {
      return log_table_[count];
    }
    return std::log(static_cast<Dtype>(1 + count));
  }

  std::vector<Dtype> log_table_;

  int width_ = 0;
  int height_ = 0;
  int range_ = 0;

  float min_height_ = 0.0;
  float max_height_ = 0.0;

  // raw feature data
  Dtype* max_height_data_ = nullptr;
  Dtype* mean_height_data_ = nullptr;
  Dtype* count_data_ = nullptr;
  Dtype* direction_data_ = nullptr;
  Dtype* top_intensity_data_ = nullptr;
  Dtype* mean_intensity_data_ = nullptr;
  Dtype* distance_data_ = nullptr;
  Dtype* nonempty_data_ = nullptr;

  // point index in feature map
  std::vector<int> map_idx_;

  // output Caffe blob
  caffe::Blob<Dtype>* out_blob_ = nullptr;
};

typedef FeatureGenerator<float> FP32FeatureGenerator;
typedef FeatureGenerator<double> FP64FeatureGenerator;

}  // namespace cnnseg
}  // namespace perception
}  // namespace apollo

#endif
