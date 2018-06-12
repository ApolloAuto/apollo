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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_COMMON_FEATURE_EXTRACTOR_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_COMMON_FEATURE_EXTRACTOR_H_

#include <memory>
#include <vector>

#include "caffe/caffe.hpp"
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/gzip_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"

#include "modules/perception/obstacle/camera/detector/common/proto/tracking_feature.pb.h"

#include "modules/common/log.h"
#include "modules/common/macro.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/camera/common/visual_object.h"

namespace apollo {
namespace perception {

class BaseFeatureExtractor {
 public:
  BaseFeatureExtractor() = default;
  virtual ~BaseFeatureExtractor() = default;
  virtual bool init(const ExtractorParam &param,
                    const boost::shared_ptr<caffe::Blob<float>> feat_blob,
                    int input_width = 0, int input_height = 0) = 0;

  // @brief: extract feature for each detected object
  // @param [in/out]: objects with bounding boxes and feature vector.
  virtual bool extract(std::vector<std::shared_ptr<VisualObject>> *objects) = 0;

 protected:
  boost::shared_ptr<caffe::Blob<float>> feat_blob_ = nullptr;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseFeatureExtractor);
};

class ReorgFeatureExtractor : public BaseFeatureExtractor {
 public:
  bool init(const ExtractorParam &param,
            const boost::shared_ptr<caffe::Blob<float>> feat_blob,
            int input_width = 0, int input_height = 0) override;
  virtual bool extract(std::vector<std::shared_ptr<VisualObject>> *objects);

 protected:
  std::vector<caffe::Blob<float> *> bottom_vec_;
  std::vector<caffe::Blob<float> *> top_vec_;
  boost::shared_ptr<caffe::Blob<float>> reorg_feat_blob_;
  boost::shared_ptr<caffe::Layer<float>> reorg_layer_ = nullptr;
  bool skip_reorg_ = false;
  int ref_height_ = 0;
  int ref_width_ = 0;
};

class ROIPoolingFeatureExtractor : public BaseFeatureExtractor {
 public:
  bool init(const ExtractorParam &param,
            const boost::shared_ptr<caffe::Blob<float>> feat_blob,
            int input_width = 0, int input_height = 0) override;
  virtual bool extract(std::vector<std::shared_ptr<VisualObject>> *objects);

 protected:
  std::vector<caffe::Blob<float> *> bottom_vec_;
  std::vector<caffe::Blob<float> *> top_vec_;
  caffe::Blob<float> rois_blob_;
  caffe::Blob<float> roi_feat_blob_;
  boost::shared_ptr<caffe::Layer<float>> roi_pooling_layer_ = nullptr;
  int input_height_ = 0;
  int input_width_ = 0;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_COMMON_FEATURE_EXTRACTOR
