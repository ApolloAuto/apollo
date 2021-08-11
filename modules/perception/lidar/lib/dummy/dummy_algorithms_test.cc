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
#include "gtest/gtest.h"

#include "modules/perception/base/point_cloud.h"
#include "modules/perception/lidar/lib/dummy/dummy_classifier.h"
#include "modules/perception/lidar/lib/dummy/dummy_ground_detector.h"
#include "modules/perception/lidar/lib/dummy/dummy_multi_target_tracker.h"
#include "modules/perception/lidar/lib/dummy/dummy_object_filter.h"
#include "modules/perception/lidar/lib/dummy/dummy_roi_filter.h"
#include "modules/perception/lidar/lib/dummy/dummy_segmentation.h"

namespace apollo {
namespace perception {
namespace lidar {

TEST(LidarDummyAlgorithmTest, lidar_dummy_algorithm_test) {
  {
    std::string name = "DummyClassifier";
    // BaseClassifier* classifier
    //   = BaseClassifierRegisterer::GetInstanceByName(name);
    BaseClassifier* classifier = new DummyClassifier();
    EXPECT_EQ(classifier->Name(), name);
    EXPECT_TRUE(classifier->Init());
    ClassifierOptions options;
    LidarFrame frame;
    EXPECT_TRUE(classifier->Classify(options, &frame));
    delete classifier;
  }
  {
    std::string name = "DummyGroundDetector";
    // BaseGroundDetector* ground_detector
    //   = BaseGroundDetectorRegisterer::GetInstanceByName(name);
    BaseGroundDetector* ground_detector = new DummyGroundDetector();
    EXPECT_EQ(ground_detector->Name(), name);
    EXPECT_TRUE(ground_detector->Init());
    GroundDetectorOptions options;
    LidarFrame frame;
    EXPECT_FALSE(ground_detector->Detect(options, nullptr));
    EXPECT_FALSE(ground_detector->Detect(options, &frame));
    frame.cloud.reset(new base::PointFCloud);
    frame.cloud->resize(2);
    EXPECT_TRUE(ground_detector->Detect(options, &frame));
    delete ground_detector;
  }
  {
    std::string name = "DummyROIFilter";
    // BaseROIFilter* roi_filter
    //   = BaseROIFilterRegisterer::GetInstanceByName(name);
    BaseROIFilter* roi_filter = new DummyROIFilter();
    EXPECT_EQ(roi_filter->Name(), name);
    EXPECT_TRUE(roi_filter->Init());
    ROIFilterOptions options;
    LidarFrame frame;
    EXPECT_FALSE(roi_filter->Filter(options, nullptr));
    EXPECT_FALSE(roi_filter->Filter(options, &frame));
    frame.cloud.reset(new base::PointFCloud);
    frame.cloud->resize(2);
    EXPECT_TRUE(roi_filter->Filter(options, &frame));
    delete roi_filter;
  }
  {
    std::string name = "DummyObjectFilter";
    // BaseObjectFilter* object_filter
    //   = BaseObjectFilterRegisterer::GetInstanceByName(name);
    BaseObjectFilter* object_filter = new DummyObjectFilter();
    EXPECT_EQ(object_filter->Name(), name);
    EXPECT_TRUE(object_filter->Init());
    ObjectFilterOptions options;
    LidarFrame frame;
    EXPECT_TRUE(object_filter->Filter(options, &frame));
    delete object_filter;
  }
  {
    std::string name = "DummySegmentation";
    // BaseSegmentation* segmentor
    //   = BaseSegmentationRegisterer::GetInstanceByName(name);
    BaseSegmentation* segmentor = new DummySegmentation();
    EXPECT_EQ(segmentor->Name(), name);
    EXPECT_TRUE(segmentor->Init());
    SegmentationOptions options;
    LidarFrame frame;
    EXPECT_TRUE(segmentor->Segment(options, &frame));
    delete segmentor;
  }
  {
    std::string name = "DummyMultiTargetTracker";
    // BaseMultiTargetTracker* tracker
    //   = BaseMultiTargetTrackerRegisterer::GetInstanceByName(name);
    BaseMultiTargetTracker* tracker = new DummyMultiTargetTracker();
    EXPECT_EQ(tracker->Name(), name);
    EXPECT_TRUE(tracker->Init());
    MultiTargetTrackerOptions options;
    EXPECT_FALSE(tracker->Track(options, nullptr));
    LidarFrame frame;
    base::ObjectPool::Instance().BatchGet(4, &frame.segmented_objects);
    for (std::size_t i = 0; i < 4; ++i) {
      frame.segmented_objects[i]->lidar_supplement.cloud.resize(1);
      frame.segmented_objects[i]->lidar_supplement.cloud[0].x =
          static_cast<float>(i);
      frame.segmented_objects[i]->lidar_supplement.cloud[0].y = 0.f;
      frame.segmented_objects[i]->lidar_supplement.cloud[0].z = 0.5f;
    }
    frame.segmented_objects[0]->direction << 0.0, 1.0, 0.0;
    frame.segmented_objects[1]->direction << 0.0, -1.0, 0.0;
    frame.segmented_objects[2]->direction << 1.0, 0.0, 0.0;
    frame.segmented_objects[3]->direction << -1.0, 0.0, 0.0;
    EXPECT_TRUE(tracker->Track(options, &frame));
    delete tracker;
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
