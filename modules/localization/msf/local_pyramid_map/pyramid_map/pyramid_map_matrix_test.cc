/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_matrix.h"

#include "gtest/gtest.h"

#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_config.h"

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

class PyramidMapMatrixTestSuite : public ::testing::Test {
 protected:
  PyramidMapMatrixTestSuite() {}
  virtual ~PyramidMapMatrixTestSuite() {}
  virtual void SetUp() {}
  virtual void TearDown() {}
};

TEST_F(PyramidMapMatrixTestSuite, constructor_and_init) {
  // map config
  std::unique_ptr<PyramidMapConfig> config(
      new PyramidMapConfig("lossy_full_alt"));
  config->SetMapNodeSize(1024, 1024);
  std::shared_ptr<PyramidMapMatrix> pm_matrix(new PyramidMapMatrix());
  pm_matrix->Init(*config);

  EXPECT_EQ(pm_matrix->rows_mr_[0], 1024);
  EXPECT_EQ(pm_matrix->cols_mr_[0], 1024);
  EXPECT_TRUE(pm_matrix->has_intensity_);
  EXPECT_TRUE(pm_matrix->has_intensity_var_);
  EXPECT_TRUE(pm_matrix->has_altitude_);
  EXPECT_TRUE(pm_matrix->has_altitude_var_);
  EXPECT_TRUE(pm_matrix->has_ground_altitude_);
  EXPECT_TRUE(pm_matrix->has_count_);
  EXPECT_TRUE(pm_matrix->has_ground_count_);
  EXPECT_EQ(pm_matrix->resolution_num_, 1);
  EXPECT_EQ(pm_matrix->ratio_, 2);

  // check operator =
  std::shared_ptr<PyramidMapMatrix> pm_matrix2(new PyramidMapMatrix());
  config->SetMapNodeSize(2, 2);
  pm_matrix2->Init(*config);
  EXPECT_EQ(pm_matrix2->rows_mr_[0], 2);
  pm_matrix2 = pm_matrix;
  EXPECT_EQ(pm_matrix2->rows_mr_[0], 1024);

  // check get row and col
  EXPECT_EQ(pm_matrix2->GetColsSafe(0), 1024);
  EXPECT_NE(pm_matrix2->GetColsSafe(1), 1024);
  EXPECT_EQ(pm_matrix2->GetRowsSafe(0), 1024);
  EXPECT_NE(pm_matrix2->GetRowsSafe(1), 1024);

  // check constructor
  std::unique_ptr<PyramidMapMatrix> pm_matrix3(
      new PyramidMapMatrix(*pm_matrix2));
}

TEST_F(PyramidMapMatrixTestSuite, get_and_set_intensity) {
  // map config
  std::unique_ptr<PyramidMapConfig> config(
      new PyramidMapConfig("lossy_full_alt"));
  config->SetMapNodeSize(2, 2);
  std::shared_ptr<PyramidMapMatrix> pm_matrix(new PyramidMapMatrix());

  // check getter safe before initializtion
  EXPECT_EQ(pm_matrix->GetIntensityMatrixSafe(), nullptr);
  EXPECT_EQ(pm_matrix->GetIntensitySafe(0, 0), nullptr);

  cv::Mat mat_intensity;
  EXPECT_FALSE(pm_matrix->GetIntensityImg(&mat_intensity));

  float intensity_data[] = {0.f, 256.f, 128.f, -1.f};
  pm_matrix->SetIntensityMatrix(intensity_data, 4, 0, 1024);
  pm_matrix->SetIntensityMatrix(intensity_data, 4, 0);
  pm_matrix->SetIntensitySafe(129.f, 0, 0, 1024);
  pm_matrix->SetIntensitySafe(129.f, 0, 0);

  // init matrix
  pm_matrix->Init(*config);

  // ensure matrix size
  EXPECT_EQ(pm_matrix->rows_mr_[0], 2);
  EXPECT_EQ(pm_matrix->cols_mr_[0], 2);

  // check getter/setter after initializtion
  pm_matrix->SetIntensityMatrix(intensity_data, 4, 0, 1024);
  pm_matrix->SetIntensityMatrix(intensity_data, 4, 0);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensity(0, 0), 0.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensity(1, 1), -1.f);

  EXPECT_EQ(pm_matrix->GetIntensitySafe(1, 2), nullptr);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensitySafe(1, 1), -1.f);

  pm_matrix->SetIntensitySafe(129.f, 0, 0, 1024);
  pm_matrix->SetIntensitySafe(129.f, 0, 0);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensitySafe(0, 0), 129.f);

  FloatMatrix* intensity_got_matrix = pm_matrix->GetIntensityMatrixSafe();
  EXPECT_FLOAT_EQ((*intensity_got_matrix)[0][0], 129.f);
  EXPECT_EQ(pm_matrix->GetIntensityMatrixSafe(1024), nullptr);

  intensity_got_matrix = pm_matrix->GetIntensityMatrix();
  EXPECT_FLOAT_EQ((*intensity_got_matrix)[1][1], -1.f);

  EXPECT_TRUE(pm_matrix->GetIntensityImg(&mat_intensity));
  EXPECT_EQ(mat_intensity.at<unsigned char>(0, 1), 255);
  EXPECT_EQ(mat_intensity.at<unsigned char>(1, 1), 0);
  EXPECT_FALSE(pm_matrix->GetIntensityImg(1024, &mat_intensity));
}

TEST_F(PyramidMapMatrixTestSuite, get_and_set_intensity_var) {
  // map config
  std::unique_ptr<PyramidMapConfig> config(
      new PyramidMapConfig("lossy_full_alt"));
  config->SetMapNodeSize(2, 2);
  std::shared_ptr<PyramidMapMatrix> pm_matrix(new PyramidMapMatrix());

  // check getter safe before initializtion
  EXPECT_EQ(pm_matrix->GetIntensityVarMatrixSafe(), nullptr);
  EXPECT_EQ(pm_matrix->GetIntensityVarSafe(0, 0), nullptr);

  float intensity_var_data[] = {0.f, 255.f, 128.f, 1.f};
  pm_matrix->SetIntensityVarMatrix(intensity_var_data, 4, 0, 1024);
  pm_matrix->SetIntensityVarMatrix(intensity_var_data, 4, 0);
  pm_matrix->SetIntensityVarSafe(129.f, 0, 0, 1024);
  pm_matrix->SetIntensityVarSafe(129.f, 0, 0);

  // init matrix
  pm_matrix->Init(*config);
  EXPECT_EQ(pm_matrix->rows_mr_[0], 2);
  EXPECT_EQ(pm_matrix->cols_mr_[0], 2);

  // check getter/setter after initializtion
  pm_matrix->SetIntensityVarMatrix(intensity_var_data, 4, 0, 1024);
  pm_matrix->SetIntensityVarMatrix(intensity_var_data, 4, 0);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensityVar(0, 0), 0.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensityVar(1, 1), 1.f);

  EXPECT_EQ(pm_matrix->GetIntensityVarSafe(1, 2), nullptr);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensityVarSafe(1, 1), 1.f);

  pm_matrix->SetIntensityVarSafe(129.f, 0, 0, 1024);
  pm_matrix->SetIntensityVarSafe(129.f, 0, 0);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensityVarSafe(0, 0), 129.f);

  FloatMatrix* intensity_var_got_matrix =
      pm_matrix->GetIntensityVarMatrixSafe();
  EXPECT_FLOAT_EQ((*intensity_var_got_matrix)[0][0], 129.f);
  FloatMatrix* intensity_var_got_matrix_level =
      pm_matrix->GetIntensityVarMatrixSafe(1024);
  EXPECT_EQ(intensity_var_got_matrix_level, nullptr);

  intensity_var_got_matrix = pm_matrix->GetIntensityVarMatrix();
  EXPECT_FLOAT_EQ((*intensity_var_got_matrix)[1][1], 1.f);
}

TEST_F(PyramidMapMatrixTestSuite, get_and_set_altitude) {
  // map config
  std::unique_ptr<PyramidMapConfig> config(
      new PyramidMapConfig("lossy_full_alt"));
  config->SetMapNodeSize(2, 2);
  std::shared_ptr<PyramidMapMatrix> pm_matrix(new PyramidMapMatrix());

  // check getter safe before initializtion
  EXPECT_EQ(pm_matrix->GetAltitudeMatrixSafe(), nullptr);
  EXPECT_EQ(pm_matrix->GetAltitudeSafe(0, 0), nullptr);

  cv::Mat mat_altitude;
  EXPECT_FALSE(pm_matrix->GetAltitudeImg(&mat_altitude));

  float altitude_data[] = {0.f, 1.f, 0.f, 1.f};
  pm_matrix->SetAltitudeMatrix(altitude_data, 4, 0, 1024);
  pm_matrix->SetAltitudeMatrix(altitude_data, 4, 0);
  pm_matrix->SetAltitudeSafe(2.f, 0, 0, 1024);
  pm_matrix->SetAltitudeSafe(2.f, 0, 0);

  // init matrix
  pm_matrix->Init(*config);
  EXPECT_EQ(pm_matrix->rows_mr_[0], 2);
  EXPECT_EQ(pm_matrix->cols_mr_[0], 2);

  // check getter/setter after initializtion
  pm_matrix->SetAltitudeMatrix(altitude_data, 4, 0, 1024);
  pm_matrix->SetAltitudeMatrix(altitude_data, 4, 0);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitude(0, 0), 0.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitude(1, 1), 1.f);

  EXPECT_EQ(pm_matrix->GetAltitudeSafe(1, 2), nullptr);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeSafe(1, 1), 1.f);

  pm_matrix->SetAltitudeSafe(2.f, 0, 0, 1024);
  pm_matrix->SetAltitudeSafe(2.f, 0, 0);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeSafe(0, 0), 2.f);

  FloatMatrix* altitude_got_matrix = pm_matrix->GetAltitudeMatrixSafe();
  EXPECT_FLOAT_EQ((*altitude_got_matrix)[0][0], 2.f);
  FloatMatrix* altitude_got_matrix_level =
      pm_matrix->GetAltitudeMatrixSafe(1024);
  EXPECT_EQ(altitude_got_matrix_level, nullptr);

  altitude_got_matrix = pm_matrix->GetAltitudeMatrix();
  EXPECT_FLOAT_EQ((*altitude_got_matrix)[1][1], 1.f);

  // case 1: count is zero
  EXPECT_TRUE(pm_matrix->GetAltitudeImg(&mat_altitude));
  EXPECT_EQ(mat_altitude.at<unsigned char>(0, 0), 0);
  EXPECT_EQ(mat_altitude.at<unsigned char>(0, 1), 0);
  EXPECT_EQ(mat_altitude.at<unsigned char>(1, 0), 0);
  EXPECT_EQ(mat_altitude.at<unsigned char>(1, 1), 0);
  EXPECT_FALSE(pm_matrix->GetAltitudeImg(1024, &mat_altitude));

  // case 2: count is not zero
  unsigned int count_data[] = {1, 1, 1, 1};
  pm_matrix->SetCountMatrix(count_data, 4, 0);
  EXPECT_TRUE(pm_matrix->GetAltitudeImg(&mat_altitude));
  EXPECT_EQ(mat_altitude.at<unsigned char>(0, 0), 255);
  EXPECT_EQ(mat_altitude.at<unsigned char>(0, 1), 127);
  EXPECT_EQ(mat_altitude.at<unsigned char>(1, 0), 0);
  EXPECT_EQ(mat_altitude.at<unsigned char>(1, 1), 127);
  EXPECT_FALSE(pm_matrix->GetAltitudeImg(1024, &mat_altitude));
}

TEST_F(PyramidMapMatrixTestSuite, get_and_set_altitude_var) {
  // map config
  std::unique_ptr<PyramidMapConfig> config(
      new PyramidMapConfig("lossy_full_alt"));
  config->SetMapNodeSize(2, 2);
  std::shared_ptr<PyramidMapMatrix> pm_matrix(new PyramidMapMatrix());

  // check getter safe before initializtion
  EXPECT_EQ(pm_matrix->GetAltitudeVarMatrixSafe(), nullptr);
  EXPECT_EQ(pm_matrix->GetAltitudeVarSafe(0, 0), nullptr);

  float altitude_var_data[] = {0.f, 256.f, 128.f, 1.f};
  pm_matrix->SetAltitudeVarMatrix(altitude_var_data, 4, 0, 1024);
  pm_matrix->SetAltitudeVarMatrix(altitude_var_data, 4, 0);
  pm_matrix->SetAltitudeVarSafe(129.f, 0, 0, 1024);
  pm_matrix->SetAltitudeVarSafe(129.f, 0, 0);

  // init matrix
  pm_matrix->Init(*config);
  EXPECT_EQ(pm_matrix->rows_mr_[0], 2);
  EXPECT_EQ(pm_matrix->cols_mr_[0], 2);

  // check getter/setter after initializtion
  pm_matrix->SetAltitudeVarMatrix(altitude_var_data, 4, 0, 1024);
  pm_matrix->SetAltitudeVarMatrix(altitude_var_data, 4, 0);

  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeVar(0, 0), 0.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeVar(1, 1), 1.f);

  EXPECT_EQ(pm_matrix->GetAltitudeVarSafe(1, 2), nullptr);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeVarSafe(1, 1), 1.f);

  pm_matrix->SetAltitudeVarSafe(129.f, 0, 0, 1024);
  pm_matrix->SetAltitudeVarSafe(129.f, 0, 0);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeVarSafe(0, 0), 129.f);

  FloatMatrix* altitude_var_got_matrix = pm_matrix->GetAltitudeVarMatrixSafe();
  EXPECT_FLOAT_EQ((*altitude_var_got_matrix)[0][0], 129.f);
  FloatMatrix* altitude_var_got_matrix_level =
      pm_matrix->GetAltitudeVarMatrixSafe(1024);
  EXPECT_EQ(altitude_var_got_matrix_level, nullptr);

  altitude_var_got_matrix = pm_matrix->GetAltitudeVarMatrix();
  EXPECT_FLOAT_EQ((*altitude_var_got_matrix)[1][1], 1.f);
}

TEST_F(PyramidMapMatrixTestSuite, get_and_set_ground_altitude) {
  // map config
  std::unique_ptr<PyramidMapConfig> config(
      new PyramidMapConfig("lossy_full_alt"));
  config->SetMapNodeSize(2, 2);
  std::shared_ptr<PyramidMapMatrix> pm_matrix(new PyramidMapMatrix());

  // check getter safe before initializtion
  EXPECT_EQ(pm_matrix->GetGroundAltitudeMatrixSafe(), nullptr);
  EXPECT_EQ(pm_matrix->GetGroundAltitudeSafe(0, 0), nullptr);

  float ground_altitude_data[] = {0.f, 1.f, 1.f, 1.f};
  pm_matrix->SetGroundAltitudeMatrix(ground_altitude_data, 4, 0, 1024);
  pm_matrix->SetGroundAltitudeMatrix(ground_altitude_data, 4, 0);
  pm_matrix->SetGroundAltitudeSafe(2.f, 0, 0, 1024);
  pm_matrix->SetGroundAltitudeSafe(2.f, 0, 0);

  // init matrix
  pm_matrix->Init(*config);
  EXPECT_EQ(pm_matrix->rows_mr_[0], 2);
  EXPECT_EQ(pm_matrix->cols_mr_[0], 2);

  // check getter/setter after initializtion
  pm_matrix->SetGroundAltitudeMatrix(ground_altitude_data, 4, 0, 1024);
  pm_matrix->SetGroundAltitudeMatrix(ground_altitude_data, 4, 0);

  EXPECT_FLOAT_EQ(*pm_matrix->GetGroundAltitude(0, 0), 0.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetGroundAltitude(1, 1), 1.f);

  EXPECT_EQ(pm_matrix->GetGroundAltitudeSafe(1, 2), nullptr);
  EXPECT_FLOAT_EQ(*pm_matrix->GetGroundAltitudeSafe(1, 1), 1.f);

  pm_matrix->SetGroundAltitudeSafe(2.f, 0, 0, 1024);
  pm_matrix->SetGroundAltitudeSafe(2.f, 0, 0);
  EXPECT_FLOAT_EQ(*pm_matrix->GetGroundAltitudeSafe(0, 0), 2.f);

  FloatMatrix* ground_altitude_got_matrix =
      pm_matrix->GetGroundAltitudeMatrixSafe();
  EXPECT_FLOAT_EQ((*ground_altitude_got_matrix)[0][0], 2.f);
  FloatMatrix* ground_altitude_got_matrix_level =
      pm_matrix->GetGroundAltitudeMatrixSafe(1024);
  EXPECT_EQ(ground_altitude_got_matrix_level, nullptr);

  ground_altitude_got_matrix = pm_matrix->GetGroundAltitudeMatrix();
  EXPECT_FLOAT_EQ((*ground_altitude_got_matrix)[1][1], 1.f);
}

TEST_F(PyramidMapMatrixTestSuite, get_and_set_count) {
  // map config
  std::unique_ptr<PyramidMapConfig> config(
      new PyramidMapConfig("lossy_full_alt"));
  config->SetMapNodeSize(2, 2);
  std::shared_ptr<PyramidMapMatrix> pm_matrix(new PyramidMapMatrix());

  // check getter safe before initializtion
  EXPECT_EQ(pm_matrix->GetCountMatrixSafe(), nullptr);
  EXPECT_EQ(pm_matrix->GetCountSafe(0, 0), nullptr);

  unsigned int count_data[] = {0, 1, 1, 1};
  pm_matrix->SetCountMatrix(count_data, 4, 0, 1024);
  pm_matrix->SetCountMatrix(count_data, 4, 0);
  pm_matrix->SetCountSafe(2, 0, 0, 1024);
  pm_matrix->SetCountSafe(2, 0, 0);

  // init matrix
  pm_matrix->Init(*config);
  EXPECT_EQ(pm_matrix->rows_mr_[0], 2);
  EXPECT_EQ(pm_matrix->cols_mr_[0], 2);

  // check getter/setter after initializtion
  pm_matrix->SetCountMatrix(count_data, 4, 0, 1024);
  pm_matrix->SetCountMatrix(count_data, 4, 0);

  EXPECT_EQ(*pm_matrix->GetCount(0, 0), 0);
  EXPECT_EQ(*pm_matrix->GetCount(1, 1), 1);

  EXPECT_EQ(pm_matrix->GetCountSafe(1, 2), nullptr);
  EXPECT_EQ(*pm_matrix->GetCountSafe(1, 1), 1);

  pm_matrix->SetCountSafe(2, 0, 0, 1024);
  pm_matrix->SetCountSafe(2, 0, 0);
  EXPECT_EQ(*pm_matrix->GetCountSafe(0, 0), 2);

  UIntMatrix* count_got_matrix = pm_matrix->GetCountMatrixSafe();
  EXPECT_EQ((*count_got_matrix)[0][0], 2);
  UIntMatrix* count_got_matrix_level = pm_matrix->GetCountMatrixSafe(1024);
  EXPECT_EQ(count_got_matrix_level, nullptr);

  count_got_matrix = pm_matrix->GetCountMatrix();
  EXPECT_EQ((*count_got_matrix)[1][1], 1);
}

TEST_F(PyramidMapMatrixTestSuite, get_and_set_ground_count) {
  // map config
  std::unique_ptr<PyramidMapConfig> config(
      new PyramidMapConfig("lossy_full_alt"));
  config->SetMapNodeSize(2, 2);
  std::shared_ptr<PyramidMapMatrix> pm_matrix(new PyramidMapMatrix());

  // check getter safe before initializtion
  EXPECT_EQ(pm_matrix->GetGroundCountMatrixSafe(), nullptr);
  EXPECT_EQ(pm_matrix->GetGroundCountSafe(0, 0), nullptr);

  unsigned int ground_count_data[] = {0, 1, 1, 1};
  pm_matrix->SetGroundCountMatrix(ground_count_data, 4, 0, 1024);
  pm_matrix->SetGroundCountMatrix(ground_count_data, 4, 0);
  pm_matrix->SetGroundCountSafe(2, 0, 0, 1024);
  pm_matrix->SetGroundCountSafe(2, 0, 0);

  // init matrix
  pm_matrix->Init(*config);
  EXPECT_EQ(pm_matrix->rows_mr_[0], 2);
  EXPECT_EQ(pm_matrix->cols_mr_[0], 2);

  // check getter/setter after initializtion
  pm_matrix->SetGroundCountMatrix(ground_count_data, 4, 0, 1024);
  pm_matrix->SetGroundCountMatrix(ground_count_data, 4, 0);

  EXPECT_EQ(*pm_matrix->GetGroundCount(0, 0), 0);
  EXPECT_EQ(*pm_matrix->GetGroundCount(1, 1), 1);

  EXPECT_EQ(pm_matrix->GetGroundCountSafe(1, 2), nullptr);
  EXPECT_EQ(*pm_matrix->GetGroundCountSafe(1, 1), 1);

  pm_matrix->SetGroundCountSafe(2, 0, 0, 1024);
  pm_matrix->SetGroundCountSafe(2, 0, 0);
  EXPECT_EQ(*pm_matrix->GetGroundCountSafe(0, 0), 2);

  UIntMatrix* ground_count_got_matrix = pm_matrix->GetGroundCountMatrixSafe();
  EXPECT_EQ((*ground_count_got_matrix)[0][0], 2);
  UIntMatrix* ground_count_got_matrix_level =
      pm_matrix->GetGroundCountMatrixSafe(1024);
  EXPECT_EQ(ground_count_got_matrix_level, nullptr);

  ground_count_got_matrix = pm_matrix->GetGroundCountMatrix();
  EXPECT_EQ((*ground_count_got_matrix)[1][1], 1);
}

TEST_F(PyramidMapMatrixTestSuite, set_and_reset_and_roi) {
  // map config
  std::unique_ptr<PyramidMapConfig> config(
      new PyramidMapConfig("lossy_full_alt"));
  config->SetMapNodeSize(3, 3);
  std::shared_ptr<PyramidMapMatrix> pm_matrix(new PyramidMapMatrix());
  pm_matrix->Init(*config);

  EXPECT_EQ(pm_matrix->rows_mr_[0], 3);
  EXPECT_EQ(pm_matrix->cols_mr_[0], 3);

  // set data
  float float_data[] = {0.f, 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f};
  unsigned int uint_data[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
  pm_matrix->SetIntensityMatrix(float_data, 9, 0);
  pm_matrix->SetIntensityVarMatrix(float_data, 9, 0);
  pm_matrix->SetAltitudeMatrix(float_data, 9, 0);
  pm_matrix->SetAltitudeVarMatrix(float_data, 9, 0);
  pm_matrix->SetGroundAltitudeMatrix(float_data, 9, 0);
  pm_matrix->SetGroundCountMatrix(uint_data, 9, 0);
  pm_matrix->SetCountMatrix(uint_data, 9, 0);

  // reset cell data
  pm_matrix->ResetCells(1, 1);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensitySafe(0, 1), 0.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensitySafe(0, 2), 2.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensityVarSafe(0, 1), 0.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeSafe(0, 1), 0.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeVarSafe(0, 1), 0.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetGroundAltitudeSafe(0, 1), 0.f);
  EXPECT_EQ(*pm_matrix->GetCountSafe(0, 1), 0);
  EXPECT_EQ(*pm_matrix->GetGroundCountSafe(0, 1), 0);

  // set ROI
  float float_data1[] = {10.f, 11.f, 12.f, 13.f, 14.f, 15.f, 16.f, 17.f, 18.f};
  FloatMatrix float_matrix;
  float_matrix.Init(3, 3);
  float_matrix.SetData(float_data1, 9, 0);

  unsigned int uint_data1[] = {10, 11, 12, 13, 14, 15, 16, 17, 18};
  UIntMatrix uint_matrix;
  uint_matrix.Init(3, 3);
  uint_matrix.SetData(uint_data1, 9, 0);

  Rect2D<unsigned int> source_roi(1, 1, 2, 2);
  Rect2D<unsigned int> target_roi(0, 0, 1, 1);
  // normal case
  pm_matrix->SetFloatMatrixRoi(&float_matrix, source_roi, target_roi, 0, 1024);
  pm_matrix->SetFloatMatrixRoi(&float_matrix, source_roi, target_roi, 0);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensitySafe(0, 0), 14.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensitySafe(1, 1), 18.f);
  pm_matrix->SetFloatMatrixRoi(&float_matrix, source_roi, target_roi, 1);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensityVarSafe(0, 0), 14.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensityVarSafe(1, 1), 18.f);

  pm_matrix->SetFloatMatrixRoi(&float_matrix, source_roi, target_roi, 2);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeSafe(0, 0), 14.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeSafe(1, 1), 18.f);
  pm_matrix->SetFloatMatrixRoi(&float_matrix, source_roi, target_roi, 3);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeVarSafe(0, 0), 14.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeVarSafe(1, 1), 18.f);

  pm_matrix->SetFloatMatrixRoi(&float_matrix, source_roi, target_roi, 4);
  EXPECT_FLOAT_EQ(*pm_matrix->GetGroundAltitudeSafe(0, 0), 14.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetGroundAltitudeSafe(1, 1), 18.f);

  pm_matrix->SetUintMatrixRoi(&uint_matrix, source_roi, target_roi, 0, 1024);
  pm_matrix->SetUintMatrixRoi(&uint_matrix, source_roi, target_roi, 0);
  EXPECT_EQ(*pm_matrix->GetCountSafe(0, 0), 14);
  EXPECT_EQ(*pm_matrix->GetCountSafe(1, 1), 18);
  pm_matrix->SetUintMatrixRoi(&uint_matrix, source_roi, target_roi, 1);
  EXPECT_EQ(*pm_matrix->GetGroundCountSafe(0, 0), 14);
  EXPECT_EQ(*pm_matrix->GetGroundCountSafe(1, 1), 18);

  // set value
  pm_matrix->SetValueSafe(12, 12.f, 1, 1);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensitySafe(1, 1), 12.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeSafe(1, 1), 12.f);

  // check check_legality_for_set_data_roi
  pm_matrix->SetUintMatrixRoi(&uint_matrix, source_roi, target_roi, 10);
  Rect2D<unsigned int> source_roi2(3, 1, 2, 2);
  pm_matrix->SetUintMatrixRoi(&uint_matrix, source_roi2, target_roi, 10);

  // base case for setting roi
  config->has_intensity_ = false;
  config->has_intensity_var_ = false;
  config->has_altitude_ = false;
  config->has_altitude_var_ = false;
  config->has_ground_altitude_ = false;
  config->has_count_ = false;
  config->has_ground_count_ = false;
  pm_matrix->Init(*config);
  pm_matrix->SetFloatMatrixRoi(&float_matrix, source_roi, target_roi, 0);
  pm_matrix->SetFloatMatrixRoi(&float_matrix, source_roi, target_roi, 1);
  pm_matrix->SetFloatMatrixRoi(&float_matrix, source_roi, target_roi, 2);
  pm_matrix->SetFloatMatrixRoi(&float_matrix, source_roi, target_roi, 3);
  pm_matrix->SetFloatMatrixRoi(&float_matrix, source_roi, target_roi, 4);
  pm_matrix->SetUintMatrixRoi(&uint_matrix, source_roi, target_roi, 0);
  pm_matrix->SetUintMatrixRoi(&uint_matrix, source_roi, target_roi, 1);

  // bad case: compute mean intensity
  EXPECT_DOUBLE_EQ(pm_matrix->ComputeMeanIntensity(0), 0.0);
}

TEST_F(PyramidMapMatrixTestSuite, merge_and_add_and_reduce) {
  // map config
  std::unique_ptr<PyramidMapConfig> config(
      new PyramidMapConfig("lossy_full_alt"));
  config->SetMapNodeSize(3, 3);
  std::shared_ptr<PyramidMapMatrix> pm_matrix(new PyramidMapMatrix());
  pm_matrix->Init(*config);

  EXPECT_EQ(pm_matrix->rows_mr_[0], 3);
  EXPECT_EQ(pm_matrix->cols_mr_[0], 3);

  // set and merge
  float float_data[] = {10.f, 10.f, 10.f, 10.f, 10.f, 10.f, 10.f, 10.f, 10.f};
  unsigned int uint_data[] = {10, 10, 10, 10, 10, 10, 10, 10, 10};
  pm_matrix->SetIntensityMatrix(float_data, 9, 0);
  pm_matrix->SetIntensityVarMatrix(float_data, 9, 0);
  pm_matrix->SetAltitudeMatrix(float_data, 9, 0);
  pm_matrix->SetAltitudeVarMatrix(float_data, 9, 0);
  pm_matrix->SetGroundAltitudeMatrix(float_data, 9, 0);
  pm_matrix->SetGroundCountMatrix(uint_data, 9, 0);
  pm_matrix->SetCountMatrix(uint_data, 9, 0);

  float intensity = 20.f;
  float intensity_var = 20.f;
  float altitude = 20.f;
  float altitude_var = 20.f;
  float ground_altitude = 20.f;
  unsigned int count = 90;
  unsigned int ground_count = 90;

  pm_matrix->MergeCellSafe(&intensity, &intensity_var, &altitude, &altitude_var,
                           &ground_altitude, &count, &ground_count, 1, 1, 0);
  // 10*0.1 + 20*0.9
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensitySafe(1, 1), 19.f);
  // 10*0.1 + 20*0.9 + (20 - 10)**2 * 0.1 * 0.9
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensityVarSafe(1, 1), 28.f);
  // 10*0.1 + 20*0.9
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeSafe(1, 1), 19.f);
  // 10*0.1 + 20*0.9 + (20 - 10)**2 * 0.1 * 0.9
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeVarSafe(1, 1), 28.f);
  // 10 + 90
  EXPECT_EQ(*pm_matrix->GetCountSafe(1, 1), 100);
  // 10 + 90
  EXPECT_EQ(*pm_matrix->GetGroundCountSafe(1, 1), 100);
  // 10*0.1 + 20*0.9
  EXPECT_FLOAT_EQ(*pm_matrix->GetGroundAltitudeSafe(1, 1), 19.f);

  // reset and mean
  pm_matrix->Reset();
  pm_matrix->SetIntensityMatrix(float_data, 9, 0);
  pm_matrix->SetIntensityVarMatrix(float_data, 9, 0);
  pm_matrix->SetAltitudeMatrix(float_data, 9, 0);
  pm_matrix->SetAltitudeVarMatrix(float_data, 9, 0);
  pm_matrix->SetGroundAltitudeMatrix(float_data, 9, 0);
  pm_matrix->SetGroundCountMatrix(uint_data, 9, 0);
  pm_matrix->SetCountMatrix(uint_data, 9, 0);

  EXPECT_DOUBLE_EQ(pm_matrix->ComputeMeanIntensity(0), 10.0);
  EXPECT_DOUBLE_EQ(pm_matrix->ComputeMeanIntensity(10), 0.0);

  // add sample
  pm_matrix->AddSampleSafe(10.f, 10.f, 1, 1, 0);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensitySafe(1, 1), 10.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensityVarSafe(1, 1), 9.090909f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeSafe(1, 1), 10.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeVarSafe(1, 1), 9.090909f);
  EXPECT_EQ(*pm_matrix->GetCountSafe(1, 1), 11);
  pm_matrix->AddSampleSafe(10.f, 10.f, 1, 1, 100);

  pm_matrix->AddGroundSample(10.f, 1, 1, 0);
  EXPECT_FLOAT_EQ(*pm_matrix->GetGroundAltitudeSafe(1, 1), 10);
  EXPECT_EQ(*pm_matrix->GetGroundCountSafe(1, 1), 11);
  pm_matrix->AddGroundSample(10.f, 1, 1, 100);

  // reset and Reduce
  pm_matrix->Reset();
  pm_matrix->SetIntensityMatrix(float_data, 9, 0);
  pm_matrix->SetIntensityVarMatrix(float_data, 9, 0);
  pm_matrix->SetAltitudeMatrix(float_data, 9, 0);
  pm_matrix->SetAltitudeVarMatrix(float_data, 9, 0);
  pm_matrix->SetGroundAltitudeMatrix(float_data, 9, 0);
  pm_matrix->SetGroundCountMatrix(uint_data, 9, 0);
  pm_matrix->SetCountMatrix(uint_data, 9, 0);

  std::shared_ptr<PyramidMapMatrix> pm_matrix2(new PyramidMapMatrix());
  pm_matrix2->Init(*config);
  pm_matrix2->SetIntensityMatrix(float_data, 9, 0);
  pm_matrix2->SetIntensityVarMatrix(float_data, 9, 0);
  pm_matrix2->SetAltitudeMatrix(float_data, 9, 0);
  pm_matrix2->SetAltitudeVarMatrix(float_data, 9, 0);
  pm_matrix2->SetGroundAltitudeMatrix(float_data, 9, 0);
  pm_matrix2->SetGroundCountMatrix(uint_data, 9, 0);
  pm_matrix2->SetCountMatrix(uint_data, 9, 0);

  PyramidMapMatrix::Reduce(pm_matrix, *pm_matrix2);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensitySafe(1, 1), 10.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensityVarSafe(1, 1), 10.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeSafe(1, 1), 10.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeVarSafe(1, 1), 10.f);
  EXPECT_EQ(*pm_matrix->GetCountSafe(1, 1), 20);
  EXPECT_FLOAT_EQ(*pm_matrix->GetGroundAltitudeSafe(1, 1), 10.f);
  EXPECT_EQ(*pm_matrix->GetGroundCountSafe(1, 1), 20);

  PyramidMapMatrix::Reduce(pm_matrix, *pm_matrix2, 1, 0);
  PyramidMapMatrix::Reduce(pm_matrix, *pm_matrix2, 0, 1);
}

TEST_F(PyramidMapMatrixTestSuite, add_merge_get_base) {
  // map config
  std::unique_ptr<PyramidMapConfig> config(
      new PyramidMapConfig("lossy_full_alt"));
  config->SetMapNodeSize(3, 3);
  std::shared_ptr<PyramidMapMatrix> pm_matrix(new PyramidMapMatrix());
  pm_matrix->Init(*config);

  EXPECT_EQ(pm_matrix->rows_mr_[0], 3);
  EXPECT_EQ(pm_matrix->cols_mr_[0], 3);

  // set
  float float_data[] = {10.f, 10.f, 10.f, 10.f, 10.f, 10.f, 10.f, 10.f, 10.f};
  unsigned int uint_data[] = {10, 10, 10, 10, 10, 10, 10, 10, 10};
  pm_matrix->SetIntensityMatrix(float_data, 9, 0);
  pm_matrix->SetIntensityVarMatrix(float_data, 9, 0);
  pm_matrix->SetAltitudeMatrix(float_data, 9, 0);
  pm_matrix->SetAltitudeVarMatrix(float_data, 9, 0);
  pm_matrix->SetGroundAltitudeMatrix(float_data, 9, 0);
  pm_matrix->SetGroundCountMatrix(uint_data, 9, 0);
  pm_matrix->SetCountMatrix(uint_data, 9, 0);

  // get
  float* intensity = nullptr;
  float* intensity_var = nullptr;
  float* altitude = nullptr;
  unsigned int* count = nullptr;
  pm_matrix->GetMapCellBase(&intensity, &intensity_var, &altitude, &count, 1, 1,
                            0);
  EXPECT_FLOAT_EQ(*intensity, 10);
  EXPECT_FLOAT_EQ(*intensity_var, 10);
  EXPECT_FLOAT_EQ(*altitude, 10);
  EXPECT_EQ(*count, 10);

  // add
  pm_matrix->AddSampleBase(10.0, 10.0, 1, 1, 0);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensitySafe(1, 1), 10.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensityVarSafe(1, 1), 9.090909f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeSafe(1, 1), 10.f);
  // EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeVarSafe(1, 1), 9.090909f);
  EXPECT_EQ(*pm_matrix->GetCountSafe(1, 1), 11);
}

TEST_F(PyramidMapMatrixTestSuite, BottomUp) {
  // map config
  std::unique_ptr<PyramidMapConfig> config(
      new PyramidMapConfig("lossy_full_alt"));
  config->SetMapNodeSize(4, 4);
  config->resolution_num_ = 2;
  std::shared_ptr<PyramidMapMatrix> pm_matrix(new PyramidMapMatrix());
  pm_matrix->Init(*config);

  EXPECT_EQ(pm_matrix->rows_mr_[0], 4);
  EXPECT_EQ(pm_matrix->cols_mr_[0], 4);
  EXPECT_EQ(pm_matrix->rows_mr_[1], 2);
  EXPECT_EQ(pm_matrix->cols_mr_[1], 2);

  // set
  float float_data[] = {10.f, 10.f, 10.f, 10.f, 10.f, 10.f, 10.f, 10.f,
                        10.f, 10.f, 10.f, 10.f, 10.f, 10.f, 10.f, 10.f};
  unsigned int uint_data[] = {10, 10, 10, 10, 10, 10, 10, 10,
                              10, 10, 10, 10, 10, 10, 10, 10};
  pm_matrix->SetIntensityMatrix(float_data, 16, 0);
  pm_matrix->SetIntensityVarMatrix(float_data, 16, 0);
  pm_matrix->SetAltitudeMatrix(float_data, 16, 0);
  pm_matrix->SetAltitudeVarMatrix(float_data, 16, 0);
  pm_matrix->SetGroundAltitudeMatrix(float_data, 16, 0);
  pm_matrix->SetGroundCountMatrix(uint_data, 16, 0);
  pm_matrix->SetCountMatrix(uint_data, 16, 0);

  // bottom up safe
  pm_matrix->BottomUpSafe();

  // check up-layer
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensitySafe(1, 1, 1), 10.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensityVarSafe(1, 1, 1), 10.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeSafe(1, 1, 1), 10.f);
  EXPECT_FLOAT_EQ(*pm_matrix->GetAltitudeVarSafe(1, 1, 1), 10.f);
  EXPECT_EQ(*pm_matrix->GetCountSafe(1, 1, 1), 40);
  EXPECT_FLOAT_EQ(*pm_matrix->GetGroundAltitudeSafe(1, 1, 1), 10.f);
  EXPECT_EQ(*pm_matrix->GetGroundCountSafe(1, 1, 1), 40);

  // bottom up base
  pm_matrix->BottomUpBase();

  // check check_legality_for_get_data
  EXPECT_EQ(pm_matrix->GetIntensitySafe(10000, 1, 1), nullptr);
  EXPECT_EQ(pm_matrix->GetIntensitySafe(1, 10000, 1), nullptr);
  EXPECT_EQ(pm_matrix->GetIntensitySafe(1, 1, 10000), nullptr);

  // check check_legality_for_set_data
  pm_matrix->SetIntensityMatrix(float_data, 16, 10000);
  pm_matrix->SetIntensityMatrix(float_data, 100000, 1);
}

TEST_F(PyramidMapMatrixTestSuite, flags_are_false_and_resolution_are_two) {
  // map config
  std::unique_ptr<PyramidMapConfig> config(
      new PyramidMapConfig("lossy_full_alt"));
  config->SetMapNodeSize(4, 4);
  config->resolution_num_ = 2;
  config->has_intensity_ = false;
  config->has_intensity_var_ = false;
  config->has_altitude_ = false;
  config->has_altitude_var_ = false;
  config->has_ground_altitude_ = false;
  config->has_count_ = false;
  config->has_ground_count_ = false;
  std::shared_ptr<PyramidMapMatrix> pm_matrix(new PyramidMapMatrix());
  pm_matrix->Init(*config);

  // bottom up safe
  pm_matrix->BottomUpSafe();

  // reset
  pm_matrix->Reset(0);
  pm_matrix->ResetCells(1, 2);
}

TEST_F(PyramidMapMatrixTestSuite, check_const_matrix) {
  // normal case
  std::unique_ptr<PyramidMapConfig> config(
      new PyramidMapConfig("lossy_full_alt"));
  config->SetMapNodeSize(2, 2);
  config->resolution_num_ = 1;
  std::shared_ptr<PyramidMapMatrix> pm_matrix(new PyramidMapMatrix());
  pm_matrix->Init(*config);

  const PyramidMapMatrix const_matrix(*pm_matrix);
  EXPECT_FLOAT_EQ((*const_matrix.GetIntensityMatrixSafe(0))[0][0], 0.f);
  EXPECT_FLOAT_EQ((*const_matrix.GetIntensityVarMatrixSafe(0))[0][0], 0.f);
  EXPECT_FLOAT_EQ((*const_matrix.GetAltitudeMatrixSafe(0))[0][0], 0.f);
  EXPECT_FLOAT_EQ((*const_matrix.GetAltitudeVarMatrixSafe(0))[0][0], 0.f);
  EXPECT_FLOAT_EQ((*const_matrix.GetGroundAltitudeMatrixSafe(0))[0][0], 0.f);
  EXPECT_EQ((*const_matrix.GetCountMatrixSafe(0))[0][0], 0);
  EXPECT_EQ((*const_matrix.GetGroundCountMatrixSafe(0))[0][0], 0);

  EXPECT_FLOAT_EQ((*const_matrix.GetIntensityMatrix(0))[0][0], 0.f);
  EXPECT_FLOAT_EQ((*const_matrix.GetIntensityVarMatrix(0))[0][0], 0.f);
  EXPECT_FLOAT_EQ((*const_matrix.GetAltitudeMatrix(0))[0][0], 0.f);
  EXPECT_FLOAT_EQ((*const_matrix.GetAltitudeVarMatrix(0))[0][0], 0.f);
  EXPECT_FLOAT_EQ((*const_matrix.GetGroundAltitudeMatrix(0))[0][0], 0.f);
  EXPECT_EQ((*const_matrix.GetCountMatrix(0))[0][0], 0);
  EXPECT_EQ((*const_matrix.GetGroundCountMatrix(0))[0][0], 0);

  // bad case 1: resolution is 1024
  std::shared_ptr<PyramidMapMatrix> pm_matrix2(new PyramidMapMatrix());
  pm_matrix2->Init(*config);
  const PyramidMapMatrix const_matrix2(*pm_matrix2);
  EXPECT_EQ(const_matrix2.GetIntensityMatrixSafe(1024), nullptr);
  EXPECT_EQ(const_matrix2.GetIntensityVarMatrixSafe(1024), nullptr);
  EXPECT_EQ(const_matrix2.GetAltitudeMatrixSafe(1024), nullptr);
  EXPECT_EQ(const_matrix2.GetAltitudeVarMatrixSafe(1024), nullptr);
  EXPECT_EQ(const_matrix2.GetGroundAltitudeMatrixSafe(1024), nullptr);
  EXPECT_EQ(const_matrix2.GetCountMatrixSafe(1024), nullptr);
  EXPECT_EQ(const_matrix2.GetGroundCountMatrixSafe(1024), nullptr);

  // bad case 2: resolution is 1, flags are false
  config->resolution_num_ = 1;
  config->has_intensity_ = false;
  config->has_intensity_var_ = false;
  config->has_altitude_ = false;
  config->has_altitude_var_ = false;
  config->has_ground_altitude_ = false;
  config->has_count_ = false;
  config->has_ground_count_ = false;
  std::shared_ptr<PyramidMapMatrix> pm_matrix3(new PyramidMapMatrix());
  pm_matrix3->Init(*config);
  const PyramidMapMatrix const_matrix3(*pm_matrix3);
  EXPECT_EQ(const_matrix3.GetIntensityMatrixSafe(0), nullptr);
  EXPECT_EQ(const_matrix3.GetIntensityVarMatrixSafe(0), nullptr);
  EXPECT_EQ(const_matrix3.GetAltitudeMatrixSafe(0), nullptr);
  EXPECT_EQ(const_matrix3.GetAltitudeVarMatrixSafe(0), nullptr);
  EXPECT_EQ(const_matrix3.GetGroundAltitudeMatrixSafe(0), nullptr);
  EXPECT_EQ(const_matrix3.GetCountMatrixSafe(0), nullptr);
  EXPECT_EQ(const_matrix3.GetGroundCountMatrixSafe(0), nullptr);
}

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
