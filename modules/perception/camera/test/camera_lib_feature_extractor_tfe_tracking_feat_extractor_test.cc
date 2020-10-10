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
#include "gtest/gtest.h"

#include "modules/perception/base/camera.h"
#include "modules/perception/base/distortion_model.h"
#include "modules/perception/camera/lib/feature_extractor/tfe/tracking_feat_extractor.h"
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/inference/utils/cuda_util.h"

namespace apollo {
namespace perception {
namespace camera {
TEST(FeatureExtractorTest, demo_test) {
  std::shared_ptr<BaseFeatureExtractor> feature_extractor_;
  FeatureExtractorInitOptions feat_options;
  feat_options.conf_file = "";
  feat_options.root_dir = "";
  feat_options.gpu_id = 0;
  std::shared_ptr<base::Blob<float>> blob_feature(new base::Blob<float>());

  blob_feature->Reshape({7, 11, 1, 1});
  feat_options.feat_blob = blob_feature;
  feat_options.input_height = 0;
  feat_options.input_width = 0;
  // feature_extractor_.reset(BaseFeatureExtractorRegisterer::GetInstanceByName(
  //    "TrackingFeatureExtractor"));
  feature_extractor_.reset(new TrackingFeatureExtractor);
  EXPECT_FALSE(feature_extractor_->Init(feat_options));
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo
