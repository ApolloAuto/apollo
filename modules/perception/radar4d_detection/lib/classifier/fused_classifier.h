/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "gtest/gtest_prod.h"

#include "modules/perception/radar4d_detection/lib/classifier/proto/fused_classifier_config.pb.h"

#include "modules/perception/common/radar/common/radar_frame.h"
#include "modules/perception/common/radar/common/object_sequence.h"
#include "modules/perception/radar4d_detection/lib/classifier/type_fusion_interface.h"
#include "modules/perception/radar4d_detection/interface/base_classifier.h"

namespace apollo {
namespace perception {
namespace radar4d {

class FusedClassifier : public BaseClassifier {
 public:
  FusedClassifier() = default;
  ~FusedClassifier() = default;
  /**
   * @brief Init fused classifier
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(
      const ClassifierInitOptions& options = ClassifierInitOptions()) override;
  /**
   * @brief Classify objects and update type info
   *
   * @param options
   * @param frame radar frame
   * @return true
   * @return false
   */
  bool Classify(const ClassifierOptions& options, RadarFrame* frame) override;
  /**
   * @brief Get class name
   *
   * @return std::string
   */
  std::string Name() const override { return "FusedClassifier"; }

 private:
  FRIEND_TEST(FusedClassifierTest, test_one_shot_fusion);
  FRIEND_TEST(FusedClassifierTest, test_one_sequence_fusion);
  FRIEND_TEST(FusedClassifierTest, test_one_sequence_fusion_bad_timestamp);
  ObjectSequence sequence_;
  double temporal_window_ = 20.0;
  bool enable_temporal_fusion_ = true;
  bool use_tracked_objects_ = true;

  std::string one_shot_fusion_method_;
  std::string sequence_fusion_method_;

  BaseOneShotTypeFusion* one_shot_fuser_;
  BaseSequenceTypeFusion* sequence_fuser_;

  std::unique_ptr<BaseOneShotTypeFusion> one_shot_fuser_ptr_;
  std::unique_ptr<BaseSequenceTypeFusion> sequence_fuser_ptr_;

  TypeFusionOption option_;
  TypeFusionInitOption init_option_;

  FusedClassifierConfig fused_classifier_config_;
};  // class FusedClassifier

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
