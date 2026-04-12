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

#include "modules/perception/common/base/object.h"

namespace apollo {
namespace perception {
namespace camera {

class FakeCalibrationService : public BaseCalibrationService {
 public:
  bool Init(const CalibrationServiceInitOptions &options =
                CalibrationServiceInitOptions()) override {
    return true;
  }

  bool BuildIndex() override { return true; }

  bool QueryGroundPlaneInCameraFrame(
      Eigen::Vector4d *plane_param) const override {
    if (plane_param != nullptr) {
      *plane_param = Eigen::Vector4d::Zero();
    }
    return false;
  }

  std::string Name() const override { return "FakeCalibrationService"; }
};

class LocationRefinerPostprocessorTest : public ::testing::Test {};

TEST_F(LocationRefinerPostprocessorTest, build_postprocessor_options_test) {
  const float bbox2d[4] = {10.0f, 20.0f, 30.0f, 40.0f};
  const float dimension_hwl[3] = {1.5f, 2.5f, 3.5f};
  const float rotation_y = 0.25f;
  const float query_plane[4] = {0.0f, 1.0f, 0.0f, -1.6f};

  auto first = LocationRefinerPostprocessor::BuildPostprocessorOptions(
      bbox2d, dimension_hwl, rotation_y, query_plane);
  auto second = LocationRefinerPostprocessor::BuildPostprocessorOptions(
      bbox2d, dimension_hwl, rotation_y, query_plane);

  ASSERT_EQ(first.line_segs.size(), 1u);
  ASSERT_EQ(second.line_segs.size(), 1u);

  EXPECT_FLOAT_EQ(first.bbox[0], bbox2d[0]);
  EXPECT_FLOAT_EQ(first.bbox[1], bbox2d[1]);
  EXPECT_FLOAT_EQ(first.bbox[2], bbox2d[2]);
  EXPECT_FLOAT_EQ(first.bbox[3], bbox2d[3]);
  EXPECT_FLOAT_EQ(first.hwl[0], dimension_hwl[0]);
  EXPECT_FLOAT_EQ(first.hwl[1], dimension_hwl[1]);
  EXPECT_FLOAT_EQ(first.hwl[2], dimension_hwl[2]);
  EXPECT_FLOAT_EQ(first.ry, rotation_y);
  EXPECT_FLOAT_EQ(first.plane[0], query_plane[0]);
  EXPECT_FLOAT_EQ(first.plane[1], query_plane[1]);
  EXPECT_FLOAT_EQ(first.plane[2], query_plane[2]);
  EXPECT_FLOAT_EQ(first.plane[3], query_plane[3]);
  EXPECT_FLOAT_EQ(first.line_segs[0].pt_start[0], bbox2d[0]);
  EXPECT_FLOAT_EQ(first.line_segs[0].pt_start[1], bbox2d[3]);
  EXPECT_FLOAT_EQ(first.line_segs[0].pt_end[0], bbox2d[2]);
  EXPECT_FLOAT_EQ(first.line_segs[0].pt_end[1], bbox2d[3]);

  first.line_segs.emplace_back(0.0f, 0.0f, 1.0f, 1.0f);

  EXPECT_EQ(first.line_segs.size(), 2u);
  EXPECT_EQ(second.line_segs.size(), 1u);
}

TEST_F(LocationRefinerPostprocessorTest,
       process_skips_when_ground_plane_unavailable_test) {
  LocationRefinerPostprocessor postprocessor;
  postprocessor.calibration_service_ =
      std::make_shared<FakeCalibrationService>();
  postprocessor.location_refiner_param_.set_min_dist_to_camera(30.0f);
  postprocessor.location_refiner_param_.set_roi_h2bottom_scale(0.5f);

  onboard::CameraFrame frame;
  auto obj = std::make_shared<base::Object>();
  obj->camera_supplement.local_center = Eigen::Vector3f(3.0f, 0.0f, 5.0f);
  obj->camera_supplement.box.xmin = 10.0f;
  obj->camera_supplement.box.ymin = 20.0f;
  obj->camera_supplement.box.xmax = 30.0f;
  obj->camera_supplement.box.ymax = 40.0f;
  obj->size = Eigen::Vector3f(4.0f, 2.0f, 1.5f);
  frame.detected_objects.push_back(obj);

  PostprocessorOptions options;
  options.do_refinement_with_calibration_service = true;

  EXPECT_TRUE(postprocessor.Process(options, &frame));
  EXPECT_EQ(frame.data_provider, nullptr);
  EXPECT_FLOAT_EQ(frame.detected_objects[0]->camera_supplement.local_center(0),
                  3.0f);
  EXPECT_FLOAT_EQ(frame.detected_objects[0]->camera_supplement.local_center(1),
                  0.0f);
  EXPECT_FLOAT_EQ(frame.detected_objects[0]->camera_supplement.local_center(2),
                  5.0f);
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
