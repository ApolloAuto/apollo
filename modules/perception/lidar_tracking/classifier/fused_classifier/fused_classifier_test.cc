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

#include <numeric>
#include <vector>
#include "gtest/gtest.h"

#include "modules/perception/common/base/object.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/common/lidar/common/lidar_frame.h"
#include "modules/perception/common/lidar/common/lidar_log.h"
#include "modules/perception/common/lidar/common/object_sequence.h"

#include "modules/perception/lidar_tracking/classifier/fused_classifier/util.h"
#include "modules/perception/lidar_tracking/classifier/fused_classifier/fused_classifier.h"

namespace apollo {
namespace perception {
namespace lidar {

class FusedClassifierTest : public testing::Test {
 protected:
  void SetUp() override {
    char cyber_path[80] = "CYBER_PATH=";
    putenv(cyber_path);
    char module_path[80] = "MODULE_PATH=";
    putenv(module_path);
    FLAGS_work_root =
        "/apollo/modules/perception/common/testdata/"
        "lidar/lib/classifier/fused_classifier";
    fused_classifier_ = new FusedClassifier();
    BuildObjects();
    for (auto& frame : frames_) {
      frame.tracked_objects = frame.segmented_objects;
    }
  }
  void TearDown() override {
    if (fused_classifier_) {
      delete fused_classifier_;
      fused_classifier_ = nullptr;
    }
  }
  void BuildObjects();
  // mapping from valid type id to type id,
  // exclude unknownunmovable and unknownmovable
  size_t IdMap(size_t i);
  float VecSum(const std::vector<float>& prob);
  void GenerateSmoothProb(std::vector<float>* prob, std::size_t id, float seed);
  void BuildBackground(base::ObjectPtr* object, int track_id);
  void BuildOneMethod(base::ObjectPtr* object, int track_id);
  void BuildNoMethod(base::ObjectPtr* object, int track_id);
  void BuildObjectsBadTimestamp();

 protected:
  FusedClassifier* fused_classifier_;
  std::vector<LidarFrame> frames_;
  std::vector<double> timestamps_;
  static const size_t kSequenceLength;
  static const size_t kObjectNum;
};

const size_t FusedClassifierTest::kSequenceLength = 10;
const size_t FusedClassifierTest::kObjectNum = 5;

size_t FusedClassifierTest::IdMap(size_t i) {
  if (i == 0) {
    return 0;  // UNKNOWN
  } else {
    return i + 2;
  }
}

float FusedClassifierTest::VecSum(const std::vector<float>& prob) {
  return std::accumulate(prob.begin(), prob.end(), 0.f);
}

void FusedClassifierTest::GenerateSmoothProb(std::vector<float>* prob,
                                             std::size_t id, float seed) {
  float p = (1.f - seed) / (VALID_OBJECT_TYPE - 1);
  for (std::size_t i = 0; i < VALID_OBJECT_TYPE; ++i) {
    prob->at(IdMap(i)) = p;
  }
  prob->at(IdMap(id)) = seed;
}

void FusedClassifierTest::BuildBackground(base::ObjectPtr* object,
                                          int track_id) {
  object->reset(new base::Object);
  (*object)->track_id = track_id;
  (*object)->lidar_supplement.is_background = 1;
}
void FusedClassifierTest::BuildOneMethod(base::ObjectPtr* object,
                                         int track_id) {
  object->reset(new base::Object);
  (*object)->track_id = track_id;
  auto& ls = (*object)->lidar_supplement;
  ls.is_background = 0;
  ls.raw_classification_methods.resize(1);
  ls.raw_classification_methods[0] = "DecisionForestClassifier";
  ls.raw_probs.resize(
      1, std::vector<float>(static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE),
                            0.f));
}
void FusedClassifierTest::BuildNoMethod(base::ObjectPtr* object, int track_id) {
  object->reset(new base::Object);
  (*object)->track_id = track_id;
  (*object)->lidar_supplement.is_background = 0;
}
void FusedClassifierTest::BuildObjectsBadTimestamp() {
  frames_.resize(kSequenceLength);
  timestamps_.resize(kSequenceLength);
  for (size_t i = 0; i < kSequenceLength; ++i) {
    frames_[i].segmented_objects.resize(kObjectNum);
    for (size_t j = 0; j < kObjectNum - 1; ++j) {
      frames_[i].segmented_objects[j].reset(new base::Object);
      frames_[i].segmented_objects[j]->track_id = static_cast<int>(j);
      auto& ls = frames_[i].segmented_objects[j]->lidar_supplement;
      ls.raw_classification_methods.resize(2);
      ls.raw_classification_methods[0] = "DecisionForestClassifier";
      ls.raw_classification_methods[1] = "CNNSegmentation";
      ls.raw_probs.resize(
          2, std::vector<float>(
                 static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
      GenerateSmoothProb(&ls.raw_probs[0], j, 0.9f);
      GenerateSmoothProb(&ls.raw_probs[1], j, 0.7f);
    }
    timestamps_[i] = static_cast<double>(i) * 0.1;
  }
  timestamps_[kSequenceLength - 1] = 0.1;
}
void FusedClassifierTest::BuildObjects() {
  frames_.resize(kSequenceLength);
  timestamps_.resize(kSequenceLength);
  for (size_t i = 0; i < kSequenceLength; ++i) {
    frames_[i].segmented_objects.resize(kObjectNum);
    for (size_t j = 0; j < kObjectNum - 1; ++j) {
      frames_[i].segmented_objects[j].reset(new base::Object);
      frames_[i].segmented_objects[j]->track_id = static_cast<int>(j);
      auto& ls = frames_[i].segmented_objects[j]->lidar_supplement;
      ls.raw_classification_methods.resize(2);
      ls.raw_classification_methods[0] = "DecisionForestClassifier";
      ls.raw_classification_methods[1] = "CNNSegmentation";
      ls.raw_probs.resize(
          2, std::vector<float>(
                 static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
      GenerateSmoothProb(&ls.raw_probs[0], j, 0.9f);
      GenerateSmoothProb(&ls.raw_probs[1], j, 0.7f);
    }
    timestamps_[i] = static_cast<double>(i) * 0.1;
  }
  BuildBackground(&frames_[0].segmented_objects[kObjectNum - 1],
                  static_cast<int>(kObjectNum - 1));
  BuildOneMethod(&frames_[1].segmented_objects[kObjectNum - 1],
                 static_cast<int>(kObjectNum));
  BuildNoMethod(&frames_[2].segmented_objects[kObjectNum - 1],
                static_cast<int>(kObjectNum - 1));
  for (size_t i = 3; i < kSequenceLength - 1; ++i) {
    BuildBackground(&frames_[i].segmented_objects[kObjectNum - 1],
                    static_cast<int>(kObjectNum - 1));
  }
  BuildOneMethod(
      &frames_[kSequenceLength - 1].segmented_objects[kObjectNum - 1],
      static_cast<int>(kObjectNum + 1));
  timestamps_[kSequenceLength - 1] =
      static_cast<double>(kSequenceLength - 1) * 0.1 + 3.0;
  timestamps_[kSequenceLength - 1] =
      static_cast<double>(kSequenceLength - 1) * 0.1 + 11.0;
}

TEST_F(FusedClassifierTest, test_basic) {
  ASSERT_TRUE(fused_classifier_ != nullptr);
  ASSERT_TRUE(fused_classifier_->Init());
  EXPECT_STREQ("FusedClassifier", fused_classifier_->Name().c_str());
}

TEST_F(FusedClassifierTest, test_object_build) {
  EXPECT_EQ(kSequenceLength, frames_.size());
  EXPECT_EQ(kSequenceLength, timestamps_.size());
  for (auto& frame : frames_) {
    EXPECT_EQ(kObjectNum, frame.segmented_objects.size());
    for (size_t i = 0; i < kObjectNum - 1; ++i) {
      auto& obj = frame.segmented_objects[i];
      EXPECT_NE(obj, nullptr);
      EXPECT_EQ(obj->lidar_supplement.raw_probs.size(), 2);
      EXPECT_EQ(obj->lidar_supplement.raw_classification_methods.size(), 2);
      EXPECT_LT(fabs(VecSum(obj->lidar_supplement.raw_probs[0]) - 1.f), 1e-6);
      EXPECT_LT(fabs(VecSum(obj->lidar_supplement.raw_probs[1]) - 1.f), 1e-6);
    }
  }
}

TEST_F(FusedClassifierTest, test_one_shot_fusion) {
  fused_classifier_->Init();
  fused_classifier_->enable_temporal_fusion_ = false;
  std::vector<BaseOneShotTypeFusion*> instances =
      BaseOneShotTypeFusionRegisterer::GetAllInstances();
  ClassifierOptions options;
  TypeFusionInitOption init_option;
  for (auto& i : instances) {
    fused_classifier_->one_shot_fuser_ = i;
    EXPECT_TRUE(fused_classifier_->one_shot_fuser_->Init(init_option));
    for (size_t n = 0; n < kSequenceLength; ++n) {
      frames_[n].timestamp = timestamps_[n];
      EXPECT_TRUE(fused_classifier_->Classify(options, &frames_[n]));
      for (size_t j = 0; j < kObjectNum - 1; ++j) {
        EXPECT_EQ(static_cast<size_t>(frames_[n].segmented_objects[j]->type),
                  IdMap(j));
      }
    }
  }
}

TEST_F(FusedClassifierTest, test_one_sequence_fusion) {
  fused_classifier_->Init();
  fused_classifier_->enable_temporal_fusion_ = true;
  std::vector<BaseSequenceTypeFusion*> instances =
      BaseSequenceTypeFusionRegisterer::GetAllInstances();
  ClassifierOptions options;
  TypeFusionInitOption init_option;
  for (auto& i : instances) {
    fused_classifier_->sequence_fuser_ = i;
    EXPECT_TRUE(fused_classifier_->sequence_fuser_->Init(init_option));
    for (size_t n = 0; n < kSequenceLength; ++n) {
      frames_[n].timestamp = timestamps_[n];
      EXPECT_TRUE(fused_classifier_->Classify(options, &frames_[n]));
      for (size_t j = 0; j < kObjectNum - 1; ++j) {
        EXPECT_EQ(static_cast<size_t>(frames_[n].segmented_objects[j]->type),
                  IdMap(j));
      }
    }
    fused_classifier_->sequence_.sequence_.clear();
  }
}

TEST_F(FusedClassifierTest, test_one_sequence_fusion_bad_timestamp) {
  BuildObjectsBadTimestamp();
  for (auto& frame : frames_) {
    frame.tracked_objects = frame.segmented_objects;
  }
  fused_classifier_->Init();
  fused_classifier_->enable_temporal_fusion_ = true;
  std::vector<BaseSequenceTypeFusion*> instances =
      BaseSequenceTypeFusionRegisterer::GetAllInstances();
  ClassifierOptions options;
  TypeFusionInitOption init_option;
  for (auto& i : instances) {
    fused_classifier_->sequence_fuser_ = i;
    EXPECT_TRUE(fused_classifier_->sequence_fuser_->Init(init_option));
    for (size_t n = 0; n < kSequenceLength; ++n) {
      frames_[n].timestamp = timestamps_[n];
      EXPECT_TRUE(fused_classifier_->Classify(options, &frames_[n]));
    }
    fused_classifier_->sequence_.sequence_.clear();
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
