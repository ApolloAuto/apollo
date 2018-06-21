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
#include "modules/perception/common/sequence_type_fuser/object_sequence.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {

class ObjectSequenceTest : public testing::Test {
 protected:
  ObjectSequenceTest() {}
  virtual ~ObjectSequenceTest() {}
  void SetUp() {}
  void TearDown() {}
  void BuildObjects();

 protected:
  ObjectSequence sequence_;
  std::vector<std::vector<std::shared_ptr<Object>>> objects_;
  std::vector<double> timestamps_;
};

void ObjectSequenceTest::BuildObjects() {
  static const int num = 10;
  objects_.resize(num);
  timestamps_.resize(num);
  for (int i = 0; i < num; ++i) {
    timestamps_[i] = static_cast<double>(i) * 0.1;
    objects_[i].resize(2);

    objects_[i][0].reset(new Object);
    objects_[i][0]->track_id = 0;

    objects_[i][1].reset(new Object);
    objects_[i][1]->track_id = i + 1;
  }
  timestamps_.back() = 5.8;
}

TEST_F(ObjectSequenceTest, TestAddAndGet) {
  BuildObjects();
  EXPECT_EQ(objects_.size(), timestamps_.size());
  for (std::size_t i = 0; i < objects_.size() - 1; ++i) {
    sequence_.AddTrackedFrameObjects(objects_[i], timestamps_[i]);
  }

  double window_time = 5.0;
  std::map<int64_t, std::shared_ptr<Object>> tracked_objects;
  sequence_.GetTrackInTemporalWindow(0, &tracked_objects, window_time);
  EXPECT_EQ(tracked_objects.size(), 9);
  sequence_.GetTrackInTemporalWindow(1, &tracked_objects, window_time);
  EXPECT_EQ(tracked_objects.size(), 1);

  sequence_.AddTrackedFrameObjects(objects_.back(), timestamps_.back());
  sequence_.GetTrackInTemporalWindow(0, &tracked_objects, window_time);
  EXPECT_EQ(tracked_objects.size(), 2);
  // window_time = 9.5;
  sequence_.GetTrackInTemporalWindow(10, &tracked_objects, window_time);
  EXPECT_EQ(tracked_objects.size(), 1);
  sequence_.GetTrackInTemporalWindow(9, &tracked_objects, window_time);
  EXPECT_EQ(tracked_objects.size(), 1);
  EXPECT_FALSE(
      sequence_.GetTrackInTemporalWindow(8, &tracked_objects, window_time));
  EXPECT_FALSE(
      sequence_.GetTrackInTemporalWindow(1, &tracked_objects, window_time));
}

}  // namespace perception
}  // namespace apollo
