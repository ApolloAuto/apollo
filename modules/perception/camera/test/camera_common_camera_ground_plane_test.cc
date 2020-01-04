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

#include "modules/perception/base/distortion_model.h"
#include "modules/perception/camera/common/camera_ground_plane.h"
#include "modules/perception/common/io/io_util.h"

namespace apollo {
namespace perception {
namespace camera {

TEST(CameraGroundPlaneTest, CameraGroundPlaneDetectorTest) {
  base::BrownCameraDistortionModel model;
  common::LoadBrownCameraIntrinsic(
      "/apollo/modules/perception/testdata/camera/common/params/"
      "onsemi_obstacle_intrinsics.yaml",
      &model);
  auto pinhole =
      static_cast<base::PinholeCameraModel *>(model.get_camera_model().get());
  Eigen::Matrix3f intrinsic = pinhole->get_intrinsic_params();
  const std::vector<float> k_mat = {
      intrinsic(0, 0), intrinsic(0, 1), intrinsic(0, 2),
      intrinsic(1, 0), intrinsic(1, 1), intrinsic(1, 2),
      intrinsic(2, 0), intrinsic(2, 1), intrinsic(2, 2)};

  const float fx = k_mat[0];
  const int img_width = 1920;
  const int img_height = 1080;
  CameraGroundPlaneDetector camera_ground_detector;
  camera_ground_detector.Init(k_mat, img_width, img_height, common::IRec(fx));

  std::vector<float> vd_samples;
  bool success = false;

  // assigned from outside
  {
    std::vector<float> plane_outside = {0.0f, 1.0f, 0.0f, -1.5f};
    success = camera_ground_detector.DetetGround(
        0.0f, 1.5f, vd_samples.data(), static_cast<int>(vd_samples.size() / 2),
        plane_outside);
    EXPECT_TRUE(success);
  }

  // lack samples
  {
    vd_samples.resize(4);
    vd_samples[0] = 500.0f;
    vd_samples[1] = 60.0f;
    vd_samples[2] = 450.0f;
    vd_samples[3] = 100.0f;
    success = camera_ground_detector.DetetGround(
        0.0f, 1.5f, vd_samples.data(), static_cast<int>(vd_samples.size() / 2));
    EXPECT_FALSE(success);
  }

  // abnormal case 1
  {
    vd_samples.clear();
    float samples[20] = {19.845f, 0.0227115f, 17.574f, 0.0221811f,
                         15.298f, 0.0221223f, 16.563f, 0.0211883f,
                         91.784f, 0.0154157f, 21.284f, 0.0243601f,
                         22.298f, 0.0247274f, 29.596f, 0.0270972f,
                         89.113f, 0.0154862f, 88.788f, 0.015517f};
    for (int i = 0; i < 10; ++i) {
      vd_samples.push_back(samples[2 * i]);
      vd_samples.push_back(samples[2 * i + 1]);
    }

    success = camera_ground_detector.DetetGround(
        0.0f, 1.5f, vd_samples.data(), static_cast<int>(vd_samples.size() / 2));
    EXPECT_FALSE(success);
  }

  // abnormal case 2
  {
    vd_samples.clear();
    float samples[20] = {1619.845f, 0.0227115f, 1617.574f, 0.0221811f,
                         1615.298f, 0.0221223f, 1616.563f, 0.0211883f,
                         1591.784f, 0.0154157f, 1621.284f, 0.0243601f,
                         1622.298f, 0.0247274f, 1629.596f, 0.0270972f,
                         1589.113f, 0.0154862f, 1588.788f, 0.015517f};
    for (int i = 0; i < 10; ++i) {
      vd_samples.push_back(samples[2 * i]);
      vd_samples.push_back(samples[2 * i + 1]);
    }

    success = camera_ground_detector.DetetGround(
        0.0f, 1.5f, vd_samples.data(), static_cast<int>(vd_samples.size() / 2));
    EXPECT_FALSE(success);
  }

  // abnormal case 3
  {
    vd_samples.clear();
    float samples[20] = {-619.845f, 0.0227115f, -617.574f, 0.0221811f,
                         -615.298f, 0.0221223f, -616.563f, 0.0211883f,
                         -591.784f, 0.0154157f, -621.284f, 0.0243601f,
                         -622.298f, 0.0247274f, -629.596f, 0.0270972f,
                         -589.113f, 0.0154862f, -588.788f, 0.015517f};
    for (int i = 0; i < 10; ++i) {
      vd_samples.push_back(samples[2 * i]);
      vd_samples.push_back(samples[2 * i + 1]);
    }

    success = camera_ground_detector.DetetGround(
        0.0f, 1.5f, vd_samples.data(), static_cast<int>(vd_samples.size() / 2));
    EXPECT_FALSE(success);
  }

  // abnormal case 4
  {
    vd_samples.clear();
    float samples[20] = {619.845f,  0.0227115f, 617.574f,  0.0221811f,
                         615.298f,  0.0221223f, 616.563f,  0.0211883f,
                         -591.784f, 0.0154157f, -621.284f, 0.0243601f,
                         -622.298f, 0.0247274f, -629.596f, 0.0270972f,
                         -589.113f, 0.0154862f, -588.788f, 0.015517f};
    for (int i = 0; i < 10; ++i) {
      vd_samples.push_back(samples[2 * i]);
      vd_samples.push_back(samples[2 * i + 1]);
    }

    success = camera_ground_detector.DetetGround(
        0.0f, 1.5f, vd_samples.data(), static_cast<int>(vd_samples.size() / 2));
    EXPECT_FALSE(success);
  }

  // normal case
  {
    vd_samples.clear();
    float samples[20] = {619.845f, 0.0227115f, 617.574f, 0.0221811f,
                         615.298f, 0.0221223f, 616.563f, 0.0211883f,
                         591.784f, 0.0154157f, 621.284f, 0.0243601f,
                         622.298f, 0.0247274f, 629.596f, 0.0270972f,
                         589.113f, 0.0154862f, 588.788f, 0.015517f};
    for (int i = 0; i < 10; ++i) {
      vd_samples.push_back(samples[2 * i]);
      vd_samples.push_back(samples[2 * i + 1]);
    }

    success = camera_ground_detector.DetetGround(
        0.0f, 1.5f, vd_samples.data(), static_cast<int>(vd_samples.size() / 2));
    EXPECT_TRUE(success);
    if (success) {
      float l[3] = {0};
      camera_ground_detector.GetGroundModel(l);
      AINFO << "#Ground estimate succeed: " << l[0] << ", " << l[1] << ", "
            << l[2];
    }
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
