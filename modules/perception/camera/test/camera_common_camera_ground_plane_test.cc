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
#include <gtest/gtest.h>
#include "modules/perception/camera/common/camera_ground_plane.h"
#include "modules/perception/base/distortion_model.h"
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
    success = camera_ground_detector.DetetGround(0.0f,
                                                 1.5f,
                                                 vd_samples.data(),
                                                 vd_samples.size() / 2,
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
    success = camera_ground_detector.DetetGround(0.0f,
                                                 1.5f,
                                                 vd_samples.data(),
                                                 vd_samples.size() / 2);
    EXPECT_FALSE(success);
  }

  // abnormal case 1
  {
    vd_samples.clear();
    float samples[20] = {
        19.845, 0.0227115,
        17.574, 0.0221811,
        15.298, 0.0221223,
        16.563, 0.0211883,
        91.784, 0.0154157,
        21.284, 0.0243601,
        22.298, 0.0247274,
        29.596, 0.0270972,
        89.113, 0.0154862,
        88.788, 0.015517};
    for (int i = 0; i < 10; ++i) {
      vd_samples.push_back(samples[2 * i]);
      vd_samples.push_back(samples[2 * i + 1]);
    }

    success = camera_ground_detector.DetetGround(0.0f,
                                                 1.5f,
                                                 vd_samples.data(),
                                                 vd_samples.size() / 2);
    EXPECT_FALSE(success);
  }

  // abnormal case 2
  {
    vd_samples.clear();
    float samples[20] = {
        1619.845, 0.0227115,
        1617.574, 0.0221811,
        1615.298, 0.0221223,
        1616.563, 0.0211883,
        1591.784, 0.0154157,
        1621.284, 0.0243601,
        1622.298, 0.0247274,
        1629.596, 0.0270972,
        1589.113, 0.0154862,
        1588.788, 0.015517};
    for (int i = 0; i < 10; ++i) {
      vd_samples.push_back(samples[2 * i]);
      vd_samples.push_back(samples[2 * i + 1]);
    }

    success = camera_ground_detector.DetetGround(0.0f,
                                                 1.5f,
                                                 vd_samples.data(),
                                                 vd_samples.size() / 2);
    EXPECT_FALSE(success);
  }

  // abnormal case 3
  {
    vd_samples.clear();
    float samples[20] = {
        -619.845, 0.0227115,
        -617.574, 0.0221811,
        -615.298, 0.0221223,
        -616.563, 0.0211883,
        -591.784, 0.0154157,
        -621.284, 0.0243601,
        -622.298, 0.0247274,
        -629.596, 0.0270972,
        -589.113, 0.0154862,
        -588.788, 0.015517};
    for (int i = 0; i < 10; ++i) {
      vd_samples.push_back(samples[2 * i]);
      vd_samples.push_back(samples[2 * i + 1]);
    }

    success = camera_ground_detector.DetetGround(0.0f,
                                                 1.5f,
                                                 vd_samples.data(),
                                                 vd_samples.size() / 2);
    EXPECT_FALSE(success);
  }

  // abnormal case 4
  {
    vd_samples.clear();
    float samples[20] = {
        619.845, 0.0227115,
        617.574, 0.0221811,
        615.298, 0.0221223,
        616.563, 0.0211883,
        -591.784, 0.0154157,
        -621.284, 0.0243601,
        -622.298, 0.0247274,
        -629.596, 0.0270972,
        -589.113, 0.0154862,
        -588.788, 0.015517};
    for (int i = 0; i < 10; ++i) {
      vd_samples.push_back(samples[2 * i]);
      vd_samples.push_back(samples[2 * i + 1]);
    }

    success = camera_ground_detector.DetetGround(0.0f,
                                                 1.5f,
                                                 vd_samples.data(),
                                                 vd_samples.size() / 2);
    EXPECT_FALSE(success);
  }

  // normal case
  {
    vd_samples.clear();
    float samples[20] = {
        619.845, 0.0227115,
        617.574, 0.0221811,
        615.298, 0.0221223,
        616.563, 0.0211883,
        591.784, 0.0154157,
        621.284, 0.0243601,
        622.298, 0.0247274,
        629.596, 0.0270972,
        589.113, 0.0154862,
        588.788, 0.015517};
    for (int i = 0; i < 10; ++i) {
      vd_samples.push_back(samples[2 * i]);
      vd_samples.push_back(samples[2 * i + 1]);
    }

    success = camera_ground_detector.DetetGround(0.0f,
                                                 1.5f,
                                                 vd_samples.data(),
                                                 vd_samples.size() / 2);
    EXPECT_TRUE(success);
    if (success) {
      float l[3] = {0};
      camera_ground_detector.GetGroundModel(l);
      AINFO << "#Ground estimate succeed: "
               << l[0] << ", "
               << l[1] << ", "
               << l[2];
    }
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
