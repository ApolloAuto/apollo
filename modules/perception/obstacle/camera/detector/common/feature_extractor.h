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

#define MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_COMMON_FEATURE_EXTRACTOR_H_

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/gzip_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <vector>

#include <caffe/caffe.hpp>
#include "modules/common/log.h

#include "modules/lib/base/noncopyable.h"
#include "modules/obstacle/base/types.h"
#include "modules/obstacle/camera/common/caffe_bridge.hpp"
#include "modules/obstacle/camera/common/visual_object.h"
#include "modules/obstacle/camera/detector/common/tracking_feature.pb.h"

namespace apollo {
namespace perception {
namespace obstacle {

class BaseFeatureExtractor {
 public:
  BaseFeatureExtractor() = default;
  virtual ~BaseFeatureExtractor() = default;
  virtual bool init(const ExtractorParam &param,
                    const boost::shared_ptr<caffe::Blob<float>> feat_blob,
                    int input_width = 0, int input_height = 0) = 0;

  virtual bool init(const ExtractorParam &param,
                    const anakin::Tensor<float> *feat_tensor,
                    int input_width = 0, int input_height = 0) {
    feat_tensor_ = feat_tensor;
    feat_blob_.reset(new caffe::Blob<float>());
    if (!sync_tensor_blob()) {
      return false;
    }
    return init(param, feat_blob_, input_width, input_height);
  }

  // @brief: extract feature for each detected object
  // @param [in/out]: objects with bounding boxes and feature vector.
  virtual bool extract(std::vector<VisualObjectPtr> *objects) = 0;

 protected:
  bool sync_tensor_blob() {
    if (feat_tensor_ == nullptr || feat_blob_ == nullptr) {
      return false;
    }
    // TODO: unit test not covered
    if (!tensor_to_blob(*feat_tensor_, feat_blob_.get())) {
      return false;
    }
    AINFO << feat_blob_->shape(0) << ", " << feat_blob_->shape(1) << ", "
          << feat_blob_->shape(2) << ", " << feat_blob_->shape(3);
    return true;
  }
  const anakin::Tensor<float> *feat_tensor_ = nullptr;
  boost::shared_ptr<caffe::Blob<float>> feat_blob_ = nullptr;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseFeatureExtractor);
};

class ReorgFeatureExtractor : public BaseFeatureExtractor {
 public:
  virtual bool init(const ExtractorParam &param,
                    const boost::shared_ptr<caffe::Blob<float>> feat_blob,
                    int input_width = 0, int input_height = 0) override;
  virtual bool extract(std::vector<VisualObjectPtr> *objects);

 protected:
  std::vector<caffe::Blob<float> *> bottom_vec_;
  std::vector<caffe::Blob<float> *> top_vec_;
  boost::shared_ptr<caffe::Blob<float>> reorg_feat_blob_;
  boost::shared_ptr<caffe::Layer<float>> reorg_layer_ = nullptr;
  bool skip_reorg_;
  int ref_height_ = 0;
  int ref_width_ = 0;
};

class ROIPoolingFeatureExtractor : public BaseFeatureExtractor {
 public:
  virtual bool init(const ExtractorParam &param,
                    const boost::shared_ptr<caffe::Blob<float>> feat_blob,
                    int input_width = 0, int input_height = 0) override;
  virtual bool extract(std::vector<VisualObjectPtr> *objects);

 protected:
  std::vector<caffe::Blob<float> *> bottom_vec_;
  std::vector<caffe::Blob<float> *> top_vec_;
  caffe::Blob<float> rois_blob_;
  caffe::Blob<float> roi_feat_blob_;
  boost::shared_ptr<caffe::Layer<float>> roi_pooling_layer_ = nullptr;
  int input_height_ = 0;
  int input_width_ = 0;
};

}  // namespace obstacle
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_COMMON_FEATURE_EXTRACTOR_H_
