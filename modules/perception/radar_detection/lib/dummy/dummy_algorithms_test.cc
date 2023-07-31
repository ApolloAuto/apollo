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
#include "modules/perception/radar_detection/lib/dummy/dummy_algorithms.h"

#include "gtest/gtest.h"
#include "modules/perception/radar_detection/common/types.h"

namespace apollo {
namespace perception {
namespace radar {

using drivers::ContiRadar;

class DummyAlgorithmsTest : public testing::Test {
 protected:
  DummyPreprocessor preprocessor;
  DummyDetector detector;
  DummyRoiFilter roi_filter;
};

ContiRadar MockContiObs() {
  ContiRadar raw_obs;

  drivers::ContiRadarObs conti_obs;
  conti_obs.set_clusterortrack(0);
  conti_obs.set_obstacle_id(80);
  conti_obs.set_longitude_dist(20);
  conti_obs.set_lateral_dist(10);
  conti_obs.set_longitude_vel(10);
  conti_obs.set_lateral_vel(5);
  conti_obs.set_rcs(15);
  conti_obs.set_dynprop(0);
  conti_obs.set_probexist(0.8);
  conti_obs.set_longitude_dist_rms(0.2);
  conti_obs.set_lateral_dist_rms(0.1);
  conti_obs.set_longitude_vel_rms(0.2);
  conti_obs.set_lateral_vel_rms(0.1);
  conti_obs.set_oritation_angle(10);
  conti_obs.set_oritation_angle_rms(2.0);
  conti_obs.set_length(2.0);
  conti_obs.set_width(1.0);
  conti_obs.set_obstacle_class(CONTI_CAR);
  conti_obs.set_meas_state(2);

  raw_obs.add_contiobs()->CopyFrom(conti_obs);
  conti_obs.set_obstacle_class(CONTI_TRUCK);
  raw_obs.add_contiobs()->CopyFrom(conti_obs);
  conti_obs.set_obstacle_class(CONTI_PEDESTRIAN);
  raw_obs.add_contiobs()->CopyFrom(conti_obs);
  conti_obs.set_obstacle_class(CONTI_MOTOCYCLE);
  raw_obs.add_contiobs()->CopyFrom(conti_obs);
  conti_obs.set_obstacle_class(CONTI_BICYCLE);
  raw_obs.add_contiobs()->CopyFrom(conti_obs);
  conti_obs.set_obstacle_class(CONTI_TYPE_UNKNOWN);
  raw_obs.add_contiobs()->CopyFrom(conti_obs);

  return raw_obs;
}

TEST_F(DummyAlgorithmsTest, dummy_test) {
  ContiRadar raw_obs = MockContiObs();
  ContiRadar corrected_obs;

  PreprocessorInitOptions preprocessor_init_options;
  bool init_result = preprocessor.Init(preprocessor_init_options);
  EXPECT_TRUE(init_result);
  EXPECT_EQ(preprocessor.Name(), "DummyPreprocessor");

  PreprocessorOptions preprocessor_options;
  preprocessor.Preprocess(raw_obs, preprocessor_options, &corrected_obs);
  EXPECT_EQ(corrected_obs.contiobs_size(), 6);
  EXPECT_EQ(corrected_obs.contiobs(0).obstacle_id(), 80);
  EXPECT_EQ(corrected_obs.contiobs(0).meas_state(), 2);

  DetectorInitOptions detector_init_options;
  init_result = detector.Init(detector_init_options);
  EXPECT_TRUE(init_result);
  EXPECT_EQ(detector.Name(), "DummyDetector");

  base::FramePtr detected_frame(new base::Frame);
  DetectorOptions detector_options;
  detector.Detect(corrected_obs, detector_options, detected_frame);
  Eigen::Vector3d center(20, 10, 0);
  Eigen::Vector3f velocity(10, 5, 0);
  EXPECT_LT((center - detected_frame->objects[0]->center).norm(), 1.0e-6);
  EXPECT_LT((velocity - detected_frame->objects[0]->velocity).norm(), 1.0e-6);

  RoiFilterInitOptions roi_filter_init_options;
  init_result = roi_filter.Init(roi_filter_init_options);
  EXPECT_TRUE(init_result);
  EXPECT_EQ(roi_filter.Name(), "DummyRoiFilter");

  RoiFilterOptions roi_filter_options;
  bool roi_filter_result =
      roi_filter.RoiFilter(roi_filter_options, detected_frame);
  EXPECT_TRUE(roi_filter_result);
  EXPECT_EQ(detected_frame->objects.size(), 6);
}

}  // namespace radar
}  // namespace perception
}  // namespace apollo
