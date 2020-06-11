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
#pragma once

#include <memory>
#include <string>

#include "gtest/gtest_prod.h"

#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/lidar/common/object_sequence.h"
#include "modules/perception/lidar/lib/classifier/fused_classifier/type_fusion_interface.h"
#include "modules/perception/lidar/lib/interface/base_classifier.h"

namespace apollo {
namespace perception {
namespace lidar {

class FusedClassifier : public BaseClassifier {
 public:
  FusedClassifier() = default;
  ~FusedClassifier() = default;
  bool Init(
      const ClassifierInitOptions& options = ClassifierInitOptions()) override;

  bool Classify(const ClassifierOptions& options, LidarFrame* frame) override;

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

  std::unique_ptr<BaseOneShotTypeFusion> one_shot_fuser_;
  std::unique_ptr<BaseSequenceTypeFusion> sequence_fuser_;

  TypeFusionOption option_;
  TypeFusionInitOption init_option_;
};  // class FusedClassifier

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
