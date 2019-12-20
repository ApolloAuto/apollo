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

#include "modules/perception/camera/lib/dummy/dummy_algorithms.h"

namespace apollo {
namespace perception {
namespace camera {

TEST(DummyObstacleTracker, test_init) {
  BaseObstacleTracker *tracker =
      BaseObstacleTrackerRegisterer::GetInstanceByName("DummyObstacleTracker");
  EXPECT_EQ("DummyObstacleTracker", tracker->Name());
  ObstacleTrackerInitOptions options;
  EXPECT_TRUE(tracker->Init(options));
}

TEST(DummyInferenceEngine, test_camera_roi_filter) {
  InferenceEngineInitOptions init_options;
  InferenceEngineOptions options;
  BaseInferenceEngine *inference_engine =
      BaseInferenceEngineRegisterer::GetInstanceByName("DummyInferenceEngine");
  EXPECT_EQ("DummyInferenceEngine", inference_engine->Name());
  EXPECT_TRUE(inference_engine->Init(init_options));

  CameraFrame frame;
  EXPECT_TRUE(inference_engine->Infer(options, &frame));
}

TEST(DummyLaneDetector, test_init) {
  BaseLaneDetector *lane_detector =
      BaseLaneDetectorRegisterer::GetInstanceByName("DummyLaneDetector");
  EXPECT_EQ("DummyLaneDetector", lane_detector->Name());
  LaneDetectorInitOptions options;
  EXPECT_TRUE(lane_detector->Init(options));
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
