/* Copyright 2019 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/

#include "modules/planning/common/history.h"

#include "cyber/common/file.h"
#include "gtest/gtest.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

class HistoryTest : public ::testing::Test {
 public:
  HistoryTest() { history_->Clear(); }

 protected:
  History* history_ = History::Instance();
};

TEST_F(HistoryTest, Add) {
  history_->Clear();

  ADCTrajectory adc_trajectory;
  EXPECT_TRUE(apollo::cyber::common::GetProtoFromFile(
      "/apollo/modules/planning/testdata/common/history_01.pb.txt",
      &adc_trajectory));
  int ret = history_->Add(adc_trajectory);
  EXPECT_EQ(0, ret);
}

TEST_F(HistoryTest, GetLastFrame) {
  FLAGS_history_max_record_num = 3;  // capacity

  history_->Clear();

  ADCTrajectory adc_trajectory_1;
  EXPECT_TRUE(apollo::cyber::common::GetProtoFromFile(
      "/apollo/modules/planning/testdata/common/history_01.pb.txt",
      &adc_trajectory_1));
  ADCTrajectory adc_trajectory_2;
  EXPECT_TRUE(apollo::cyber::common::GetProtoFromFile(
      "/apollo/modules/planning/testdata/common/history_02.pb.txt",
      &adc_trajectory_2));

  // seq_num: 1
  history_->Add(adc_trajectory_1);
  EXPECT_NE(nullptr, history_->GetLastFrame());
  EXPECT_EQ(1, history_->Size());
  EXPECT_EQ(1, history_->GetLastFrame()->seq_num());

  // seq_num: 2
  history_->Add(adc_trajectory_2);
  EXPECT_NE(nullptr, history_->GetLastFrame());
  EXPECT_EQ(2, history_->Size());
  EXPECT_EQ(2, history_->GetLastFrame()->seq_num());

  // seq_num: 1
  history_->Add(adc_trajectory_1);
  EXPECT_NE(nullptr, history_->GetLastFrame());
  EXPECT_EQ(3, history_->Size());
  EXPECT_EQ(1, history_->GetLastFrame()->seq_num());

  // seq_num: 2
  history_->Add(adc_trajectory_2);
  EXPECT_NE(nullptr, history_->GetLastFrame());
  EXPECT_EQ(3, history_->Size());
  EXPECT_EQ(2, history_->GetLastFrame()->seq_num());
}

TEST_F(HistoryTest, GetObjectDecisions) {
  history_->Clear();

  ADCTrajectory adc_trajectory;
  EXPECT_TRUE(apollo::cyber::common::GetProtoFromFile(
      "/apollo/modules/planning/testdata/common/history_01.pb.txt",
      &adc_trajectory));

  history_->Add(adc_trajectory);
  EXPECT_NE(nullptr, history_->GetLastFrame());
  std::vector<const HistoryObjectDecision*> object_decisions =
      history_->GetLastFrame()->GetObjectDecisions();
  EXPECT_EQ(3, object_decisions.size());

  // sort
  std::sort(
      object_decisions.begin(), object_decisions.end(),
      [](const HistoryObjectDecision* lhs, const HistoryObjectDecision* rhs) {
        return lhs->id() < rhs->id();
      });

  for (const HistoryObjectDecision* object_decision : object_decisions) {
    ADEBUG << "object_decision[" << object_decision->id() << "]";
    auto obj_decision = object_decision->GetObjectDecision();
    for (const ObjectDecisionType* decision_type : obj_decision) {
      ADEBUG << "  decistion_type[" << decision_type->object_tag_case() << "]";
    }
  }

  // 11720: nudge, stop
  EXPECT_STREQ("11720", object_decisions[0]->id().c_str());
  auto obj_decision = object_decisions[0]->GetObjectDecision();
  EXPECT_EQ(2, obj_decision.size());
  // sort
  std::sort(obj_decision.begin(), obj_decision.end(),
            [](const ObjectDecisionType* lhs, const ObjectDecisionType* rhs) {
              return lhs->object_tag_case() < rhs->object_tag_case();
            });
  EXPECT_TRUE(obj_decision[0]->has_stop());
  EXPECT_TRUE(obj_decision[1]->has_nudge());

  // CW_2832: stop
  EXPECT_STREQ("CW_2832", object_decisions[1]->id().c_str());
  obj_decision = object_decisions[1]->GetObjectDecision();
  EXPECT_EQ(1, obj_decision.size());
  EXPECT_TRUE(obj_decision[0]->has_stop());

  // TL_2516: stop
  EXPECT_STREQ("TL_2516", object_decisions[2]->id().c_str());
  obj_decision = object_decisions[2]->GetObjectDecision();
  EXPECT_EQ(1, obj_decision.size());
  EXPECT_TRUE(obj_decision[0]->has_stop());
}

TEST_F(HistoryTest, GetStopObjectDecisions) {
  history_->Clear();

  ADCTrajectory adc_trajectory;
  EXPECT_TRUE(apollo::cyber::common::GetProtoFromFile(
      "/apollo/modules/planning/testdata/common/history_01.pb.txt",
      &adc_trajectory));

  history_->Add(adc_trajectory);
  EXPECT_NE(nullptr, history_->GetLastFrame());
  std::vector<const HistoryObjectDecision*> object_decisions =
      history_->GetLastFrame()->GetStopObjectDecisions();
  EXPECT_EQ(3, object_decisions.size());

  // 11720: stop
  EXPECT_STREQ("11720", object_decisions[0]->id().c_str());
  auto obj_decision = object_decisions[0]->GetObjectDecision();
  EXPECT_EQ(1, obj_decision.size());
  EXPECT_TRUE(obj_decision[0]->has_stop());

  // CW_2832: stop
  EXPECT_STREQ("CW_2832", object_decisions[1]->id().c_str());
  obj_decision = object_decisions[1]->GetObjectDecision();
  EXPECT_EQ(1, obj_decision.size());
  EXPECT_TRUE(obj_decision[0]->has_stop());

  // TL_2516: stop
  EXPECT_STREQ("TL_2516", object_decisions[2]->id().c_str());
  obj_decision = object_decisions[2]->GetObjectDecision();
  EXPECT_EQ(1, obj_decision.size());
  EXPECT_TRUE(obj_decision[0]->has_stop());
}

TEST_F(HistoryTest, GetObjectDecisionsById) {
  history_->Clear();

  ADCTrajectory adc_trajectory;
  EXPECT_TRUE(apollo::cyber::common::GetProtoFromFile(
      "/apollo/modules/planning/testdata/common/history_01.pb.txt",
      &adc_trajectory));
  history_->Add(adc_trajectory);
  EXPECT_NE(nullptr, history_->GetLastFrame());

  // 11720: nudge, stop
  const HistoryObjectDecision* object_decision =
      history_->GetLastFrame()->GetObjectDecisionsById("11720");
  EXPECT_STREQ("11720", object_decision->id().c_str());
  auto obj_decision = object_decision->GetObjectDecision();
  // sort
  std::sort(obj_decision.begin(), obj_decision.end(),
            [](const ObjectDecisionType* lhs, const ObjectDecisionType* rhs) {
              return lhs->object_tag_case() < rhs->object_tag_case();
            });
  EXPECT_EQ(2, obj_decision.size());
  EXPECT_TRUE(obj_decision[0]->has_stop());
  EXPECT_TRUE(obj_decision[1]->has_nudge());
}

}  // namespace planning
}  // namespace apollo
