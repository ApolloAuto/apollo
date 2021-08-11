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

#include "modules/perception/camera/common/twod_threed_util.h"

namespace apollo {
namespace perception {
namespace camera {

TEST(TwodThreedUtilTest, CalAngleDiffTest) {
  const float eps = 1e-5f;
  const float M_PI_float = static_cast<float>(M_PI);

  float a1 = 0.f;
  float a2 = M_PI_float * 0.5f;
  EXPECT_NEAR(CalAngleDiff(a1, a2), M_PI_float * 0.5f, eps);

  a1 = M_PI_float * 0.5f;
  a2 = 0.f;
  EXPECT_NEAR(CalAngleDiff(a1, a2), M_PI_float * 0.5f, eps);

  a1 = 0.f;
  a2 = M_PI_float;
  EXPECT_NEAR(CalAngleDiff(a1, a2), M_PI_float, eps);

  a1 = M_PI_float;
  a2 = 0.f;
  EXPECT_NEAR(CalAngleDiff(a1, a2), M_PI_float, eps);

  a1 = -M_PI_float;
  a2 = M_PI_float;
  EXPECT_NEAR(CalAngleDiff(a1, a2), 0.f, eps);

  a1 = M_PI_float;
  a2 = -M_PI_float;
  EXPECT_NEAR(CalAngleDiff(a1, a2), 0.f, eps);

  a1 = 0.f;
  a2 = 2.f * M_PI_float + 0.2f;
  EXPECT_NEAR(CalAngleDiff(a1, a2), 0.2f, eps);

  a1 = 2.f * M_PI_float + 0.2f;
  a2 = 0.f;
  EXPECT_NEAR(CalAngleDiff(a1, a2), 0.2f, eps);
}

TEST(TwodThreedUtilTest, GetSharpAngleTest) {
  const float eps = 1e-5f;
  const float M_PI_float = static_cast<float>(M_PI);

  float a1 = 0.f;
  float a2 = M_PI_float * 0.75f;
  EXPECT_NEAR(GetSharpAngle(a1, a2), M_PI_float * 0.25f, eps);

  a1 = M_PI_float * 0.75f;
  a2 = 0.f;
  EXPECT_NEAR(GetSharpAngle(a1, a2), M_PI_float * 0.25f, eps);

  a1 = 0.f;
  a2 = M_PI_float * 0.25f;
  EXPECT_NEAR(GetSharpAngle(a1, a2), M_PI_float * 0.25f, eps);

  a1 = M_PI_float * 0.25f;
  a2 = 0.f;
  EXPECT_NEAR(GetSharpAngle(a1, a2), M_PI_float * 0.25f, eps);
}

TEST(TwodThreedUtilTest, GetJaccardIndexTest) {
  const float eps = 1e-5f;

  float bbox_ref[4] = {200, 200, 400, 400};
  {
    float bbox[4] = {0, 0, 100, 100};
    EXPECT_NEAR(GetJaccardIndex(bbox_ref, bbox[0], bbox[1], bbox[2], bbox[3]),
                0.f, eps);
  }
  {
    float bbox[4] = {200, 0, 200, 100};
    EXPECT_NEAR(GetJaccardIndex(bbox_ref, bbox[0], bbox[1], bbox[2], bbox[3]),
                0.f, eps);
  }
  {
    float bbox[4] = {0, 200, 100, 400};
    EXPECT_NEAR(GetJaccardIndex(bbox_ref, bbox[0], bbox[1], bbox[2], bbox[3]),
                0.f, eps);
  }
  {
    float bbox[4] = {200, 200, 300, 300};
    EXPECT_NEAR(GetJaccardIndex(bbox_ref, bbox[0], bbox[1], bbox[2], bbox[3]),
                0.25f, eps);
  }
}

TEST(TwodThreedUtilTest, OccludeTest) {
  {
    float bbox1[4] = {200, 200, 400, 400};
    float bbox2[4] = {0, 0, 100, 100};
    float h1 = 2.0f;
    float h2 = 2.0f;
    EXPECT_FALSE(Occlude(bbox1, h1, bbox2, h2));
  }
  {
    float bbox1[4] = {200, 200, 400, 400};
    float bbox2[4] = {200, 0, 200, 100};
    float h1 = 2.0f;
    float h2 = 2.0f;
    EXPECT_FALSE(Occlude(bbox1, h1, bbox2, h2));
  }
  {
    float bbox1[4] = {200, 200, 400, 400};
    float bbox2[4] = {0, 200, 100, 400};
    float h1 = 2.0f;
    float h2 = 2.0f;
    EXPECT_FALSE(Occlude(bbox1, h1, bbox2, h2));
  }
  {
    float bbox1[4] = {200, 200, 400, 400};
    float bbox2[4] = {200, 200, 300, 300};
    float h1 = 2.0f;
    float h2 = 2.0f;
    EXPECT_TRUE(Occlude(bbox1, h1, bbox2, h2));
  }
  {
    float bbox1[4] = {200, 200, 400, 400};
    float bbox2[4] = {200, 200, 300, 300};
    float h1 = 2.0f;
    float h2 = 0.00001f;
    EXPECT_FALSE(Occlude(bbox1, h1, bbox2, h2));
  }
  {
    float bbox1[4] = {200, 200, 201, 201};
    float bbox2[4] = {200, 200, 300, 300};
    float h1 = 2.0f;
    float h2 = 2.0f;
    EXPECT_FALSE(Occlude(bbox1, h1, bbox2, h2));
  }
  {
    float bbox1[4] = {200, 200, 201, 201};
    float bbox2[4] = {200, 200, 300, 300};
    float h1 = 10.0f;
    float h2 = 2.0f;
    EXPECT_FALSE(Occlude(bbox1, h1, bbox2, h2));
  }
}

TEST(TwodThreedUtilTest, GetBboxFromPtsTest) {
  const float eps = 1e-5f;
  {
    float pts[] = {0.5f, 0.6f};
    float bbox[4] = {0.f};
    GetBboxFromPts(pts, 1, bbox);
    EXPECT_NEAR(bbox[0], pts[0], eps);
    EXPECT_NEAR(bbox[1], pts[1], eps);
    EXPECT_NEAR(bbox[2], pts[0], eps);
    EXPECT_NEAR(bbox[3], pts[1], eps);
  }
  {
    float pts[] = {0.5f, 0.6f, 1.f, 2.f};
    float bbox[4] = {0.f};
    GetBboxFromPts(pts, 2, bbox);
    EXPECT_NEAR(bbox[0], pts[0], eps);
    EXPECT_NEAR(bbox[1], pts[1], eps);
    EXPECT_NEAR(bbox[2], pts[2], eps);
    EXPECT_NEAR(bbox[3], pts[3], eps);
  }
}

TEST(TwodThreedUtilTest, GetMinIndexVecTest) {
  {
    float vec[] = {-1.f};
    EXPECT_EQ(GetMinIndexVec(vec, 1), 0);
  }
  {
    float vec[] = {10.f, -1.f, 0.f, 2.2f, -2.f, 5.f};
    EXPECT_EQ(GetMinIndexVec(vec, 6), 4);
  }
}

TEST(TwodThreedUtilTest, CheckXYTest) {
  const int width = 100;
  const int height = 100;

  int x = 10;
  int y = 10;
  EXPECT_TRUE(CheckXY(x, y, width, height));

  x = -1;
  y = -1;
  EXPECT_FALSE(CheckXY(x, y, width, height));

  x = -1;
  y = 10;
  EXPECT_FALSE(CheckXY(x, y, width, height));

  x = 10;
  y = -1;
  EXPECT_FALSE(CheckXY(x, y, width, height));

  x = 200;
  y = 200;
  EXPECT_FALSE(CheckXY(x, y, width, height));

  x = 10;
  y = 200;
  EXPECT_FALSE(CheckXY(x, y, width, height));

  x = 200;
  y = 10;
  EXPECT_FALSE(CheckXY(x, y, width, height));
}

TEST(TwodThreedUtilTest, GetScoreViaRotDimensionCenterTest) {
  float bbox_ref[4] = {0, 0, 0, 0};
  float hwl8[3] = {1.5f, 1.6f, 4.0f};
  float object_center1[3] = {-3.32887f, 1.86078f, 17.2908f};

  float k_mat[9] = {2022.5639693330495f,
                    0.0f,
                    989.3893672805314f,
                    0.0f,
                    2014.0535251884398f,
                    570.6145712367711f,
                    0.0f,
                    0.0f,
                    1.0f};

  float rotation_y = 1.5f;
  float rot[9];
  rot[0] = static_cast<float>(cos(rotation_y));
  rot[2] = static_cast<float>(sin(rotation_y));
  rot[4] = 1.0f;
  rot[6] = static_cast<float>(-sin(rotation_y));
  rot[8] = static_cast<float>(cos(rotation_y));
  rot[1] = rot[3] = rot[5] = rot[7] = static_cast<float>(0);

  int image_width = 1920;
  int image_height = 1080;

  float bbox_res[4];
  float bbox_near[4];

  bool check_truncation = false;
  float score = GetScoreViaRotDimensionCenter(k_mat, image_width, image_height,
                                              bbox_ref, rot, hwl8,
                                              object_center1, check_truncation);
  EXPECT_NEAR(score, 0.f, 1e-5);

  check_truncation = true;
  score = GetScoreViaRotDimensionCenter(k_mat, image_width, image_height,
                                        bbox_ref, rot, hwl8, object_center1,
                                        check_truncation);
  EXPECT_NEAR(score, 0.f, 1e-5);

  score = GetScoreViaRotDimensionCenter(k_mat, image_width, image_height,
                                        bbox_ref, rot, hwl8, object_center1,
                                        check_truncation, bbox_res, bbox_near);
  EXPECT_NEAR(score, 0.f, 1e-5);
}

TEST(TwodThreedUtilTest, GetDxDzForCenterFromGroundLineSegTest) {
  {
    int width = 1920;
    int height = 1080;
    float k_mat[9] = {1975.43f, 0.0f, 958.069f, 0.0f, 1979.28f,
                      460.072f, 0.0f, 0.0f,     1.0f};
    LineSegment2D<float> ls(0.0f, 0.0f, 0.1f, 0.0f);
    float plane[4] = {0};
    float pts_c[12] = {0};
    float ratio_x_over_z = 1.0f;
    float dx_dz[2] = {0};
    GetDxDzForCenterFromGroundLineSeg(ls, plane, pts_c, k_mat, width, height,
                                      ratio_x_over_z, dx_dz, true);
    EXPECT_LT(dx_dz[1], 1e-5);
  }
  {
    int width = 1920;
    int height = 1080;
    float k_mat[9] = {1975.43f, 0.0f, 958.069f, 0.0f, 1979.28f,
                      460.072f, 0.0f, 0.0f,     1.0f};
    LineSegment2D<float> ls(200, 400, 500, 400);
    float plane[4] = {0.0f, 0.0f, 1.0f, 10.0f};
    float pts_c[12] = {0};
    float ratio_x_over_z = 1.0f;
    float dx_dz[2] = {0};
    GetDxDzForCenterFromGroundLineSeg(ls, plane, pts_c, k_mat, width, height,
                                      ratio_x_over_z, dx_dz, true);
    EXPECT_LT(dx_dz[1], 1e-5);
  }
  {
    int width = 1193;
    int height = 712;
    float k_mat[9] = {1447.44f, 0.0f, 495.503f, 0.0f, 1446.67f,
                      381.524f, 0.0f, 0.0f,     1.0f};
    LineSegment2D<float> ls(165.481f, 482.044f, 580.937f, 482.044f);
    float plane[4] = {0.0f, 1.0f, 0.0f, -1.5f};
    float pts_c[12] = {7.03179f,  0.281616f, 13.5518f,  5.49636f,
                       0.281616f, 13.5518f,  5.49636f,  0.281616f,
                       17.5286f,  7.03179f,  0.281616f, 17.5286f};
    float ratio_x_over_z = 0.403088f;
    float dx_dz[2] = {0};
    GetDxDzForCenterFromGroundLineSeg(ls, plane, pts_c, k_mat, width, height,
                                      ratio_x_over_z, dx_dz, true);
    EXPECT_GE(dx_dz[1], 1e-5);
    GetDxDzForCenterFromGroundLineSeg(ls, plane, pts_c, k_mat, width, height,
                                      ratio_x_over_z, dx_dz, false);
    EXPECT_GE(dx_dz[1], 1e-5);
  }
  {
    int width = 1193;
    int height = 712;
    float k_mat[9] = {1447.44f, 0.0f, 495.503f, 0.0f, 1446.67f,
                      381.524f, 0.0f, 0.0f,     1.0f};
    LineSegment2D<float> ls(271.519f, 393.128f, 990.795f, 393.128f);
    float plane[4] = {0.0f, 1.0f, 0.0f, -1.5f};
    float pts_c[12] = {5.74431f, 0.84582f, 11.623f,  6.78823f,
                       0.84582f, 10.1878f, 2.88452f, 0.84582f,
                       7.34827f, 1.8406f,  0.84582f, 8.78343f};
    float ratio_x_over_z = 0.454838f;
    float dx_dz[2] = {0};
    GetDxDzForCenterFromGroundLineSeg(ls, plane, pts_c, k_mat, width, height,
                                      ratio_x_over_z, dx_dz, true);
    EXPECT_GE(dx_dz[1], 1e-5);
    GetDxDzForCenterFromGroundLineSeg(ls, plane, pts_c, k_mat, width, height,
                                      ratio_x_over_z, dx_dz, false);
    EXPECT_GE(dx_dz[1], 1e-5);
  }
  {
    int width = 1193;
    int height = 712;
    float k_mat[9] = {1447.44f, 0.0f, 495.503f, 0.0f, 1446.67f,
                      381.524f, 0.0f, 0.0f,     1.0f};
    LineSegment2D<float> ls(424.105f, 383.528f, 1106.85f, 383.528f);
    float plane[4] = {0.0f, 1.0f, 0.0f, -0.0159766f};
    float pts_c[12] = {2.49833f,   0.0501389f, 21.9282f,   3.86523f,
                       0.0501389f, 21.4518f,   2.33395f,   0.0501389f,
                       17.058f,    0.967046f,  0.0501389f, 17.5344f};
    float ratio_x_over_z = 0.123948f;
    float dx_dz[2] = {0};
    GetDxDzForCenterFromGroundLineSeg(ls, plane, pts_c, k_mat, width, height,
                                      ratio_x_over_z, dx_dz, false);
    EXPECT_LT(dx_dz[1], -1e-5);
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
