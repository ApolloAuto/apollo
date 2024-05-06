/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/perception/camera_tracking/proto/tracking_feature.pb.h"

#include "modules/perception/camera_tracking/common/camera_tracking_frame.h"
#include "modules/perception/camera_tracking/interface/base_feature_extractor.h"
#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/base/object.h"
#include "modules/perception/common/camera/common/util.h"
#include "modules/perception/common/inference/operators/roipooling_layer.h"
#include "modules/perception/common/inference/utils/gemm.h"

namespace apollo {
namespace perception {
namespace camera {

struct FeatureExtractorLayer {
  std::shared_ptr<inference::Layer<float>> pooling_layer;
  std::shared_ptr<base::Blob<float>> rois_blob;
  std::shared_ptr<base::Blob<float>> top_blob;
};

class TrackingFeatureExtractor : public BaseFeatureExtractor {
 public:
  TrackingFeatureExtractor() = default;
  ~TrackingFeatureExtractor() = default;

  /**
   * @brief init feature extractor
   * 
   * @param init_options 
   * @return true 
   * @return false 
   */
  bool Init(const FeatureExtractorInitOptions &init_options) override;
  /**
   * @brief extract feature from frame
   * 
   * @param options 
   * @param frame 
   * @return true 
   * @return false 
   */
  bool Extract(const FeatureExtractorOptions &options,
               CameraTrackingFrame *frame) override;

 protected:
  void init_roipooling(const tracking_feature::ROIPoolingParam &param);
  std::vector<std::shared_ptr<FeatureExtractorLayer>> roi_poolings_;
  inference::GPUL2Norm norm_;
  int input_height_ = 0;
  int input_width_ = 0;
  int feat_width_ = 0;
  int feat_height_ = 0;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
