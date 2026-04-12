/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/camera_location_refinement/location_refiner/location_refiner_postprocessor.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace camera {

class LocationRefinerPostprocessorTest : public ::testing::Test {};

TEST_F(LocationRefinerPostprocessorTest, roi_is_symmetric_test) {
  LocationRefinerPostprocessor postprocessor;

  constexpr float kImageWidth = 1920.0f;
  constexpr float kImageHeight = 1080.0f;
  constexpr float kVanishingRow = 540.0f;
  constexpr float kBottomMargin = 270.0f;
  constexpr float kSampleRow = 675.0f;

  const float inside_left[2] = {500.0f, kSampleRow};
  const float inside_right[2] = {kImageWidth - inside_left[0], kSampleRow};
  const float outside_left[2] = {420.0f, kSampleRow};
  const float outside_right[2] = {kImageWidth - outside_left[0], kSampleRow};

  EXPECT_TRUE(postprocessor.is_in_roi(inside_left, kImageWidth, kImageHeight,
                                      kVanishingRow, kBottomMargin));
  EXPECT_TRUE(postprocessor.is_in_roi(inside_right, kImageWidth, kImageHeight,
                                      kVanishingRow, kBottomMargin));
  EXPECT_FALSE(postprocessor.is_in_roi(outside_left, kImageWidth,
                                       kImageHeight, kVanishingRow,
                                       kBottomMargin));
  EXPECT_FALSE(postprocessor.is_in_roi(outside_right, kImageWidth,
                                       kImageHeight, kVanishingRow,
                                       kBottomMargin));
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
