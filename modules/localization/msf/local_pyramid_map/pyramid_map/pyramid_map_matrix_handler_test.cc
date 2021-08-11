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
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_matrix_handler.h"

#include <memory>
#include "gtest/gtest.h"

#include "modules/localization/msf/local_pyramid_map/base_map/base_map_matrix_handler.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_matrix.h"

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

class PyramidMapMatrixHandlerTestSuite : public ::testing::Test {
 protected:
  PyramidMapMatrixHandlerTestSuite() {}
  virtual ~PyramidMapMatrixHandlerTestSuite() {}
  virtual void SetUp() {}
  virtual void TearDown() {}
};

TEST_F(PyramidMapMatrixHandlerTestSuite, LossyMapFullAltMatrixHandler) {
  // create handler
  BaseMapMatrixHandler* handler =
      PyramidMapMatrixHandlerSelector::AllocPyramidMapMatrixHandler(
          MapVersion::LOSSY_FULL_ALT_MAP);

  // create pyramid map matrix
  std::unique_ptr<PyramidMapConfig> config(
      new PyramidMapConfig("lossy_full_alt"));
  config->SetMapNodeSize(2, 2);
  config->resolution_num_ = 1;
  std::shared_ptr<PyramidMapMatrix> pm_matrix(new PyramidMapMatrix());
  pm_matrix->Init(*config);

  // set matrix data
  float float_data[] = {0.f, 256.f, 10.f, 10.f};
  unsigned int uint_data[] = {10, 10, 10, 10};
  pm_matrix->SetIntensityMatrix(float_data, 4, 0);
  pm_matrix->SetIntensityVarMatrix(float_data, 4, 0);
  pm_matrix->SetAltitudeMatrix(float_data, 4, 0);
  pm_matrix->SetAltitudeVarMatrix(float_data, 4, 0);
  pm_matrix->SetGroundAltitudeMatrix(float_data, 4, 0);
  pm_matrix->SetGroundCountMatrix(uint_data, 4, 0);
  pm_matrix->SetCountMatrix(uint_data, 4, 0);

  // normal case: check get/create/load
  size_t expect_size = sizeof(unsigned int) * 2 + sizeof(float) * 4;
  expect_size += pm_matrix->GetRowsSafe() * pm_matrix->GetColsSafe() *
                 (sizeof(unsigned char) + sizeof(unsigned char) +
                  sizeof(uint16_t) + sizeof(uint16_t) + sizeof(uint16_t));
  std::shared_ptr<unsigned char> buf(new unsigned char[expect_size]);
  EXPECT_EQ(handler->GetBinarySize(pm_matrix), expect_size);
  EXPECT_EQ(handler->CreateBinary(pm_matrix, buf.get(), expect_size - 1), 0);
  EXPECT_EQ(handler->CreateBinary(pm_matrix, buf.get(), expect_size),
            expect_size);
  EXPECT_EQ(handler->LoadBinary(buf.get(), pm_matrix), expect_size);

  // check the matrix after save and load
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensitySafe(0, 1), 255.f);

  // bad case 1: flags are false
  config->has_intensity_ = false;
  config->has_intensity_var_ = false;
  config->has_altitude_ = false;
  config->has_altitude_var_ = false;
  config->has_ground_altitude_ = false;
  config->has_ground_count_ = false;
  config->has_count_ = false;
  pm_matrix->Init(*config);
  expect_size = 56;
  EXPECT_EQ(handler->GetBinarySize(pm_matrix), expect_size);
  EXPECT_EQ(handler->CreateBinary(pm_matrix, buf.get(), expect_size - 1), 0);
  EXPECT_EQ(handler->CreateBinary(pm_matrix, buf.get(), expect_size),
            expect_size);
  EXPECT_EQ(handler->LoadBinary(buf.get(), pm_matrix), expect_size);

  if (handler != nullptr) {
    delete handler;
    handler = nullptr;
  }
}

TEST_F(PyramidMapMatrixHandlerTestSuite, LosslessMapMatrixHandler) {
  // create handler
  BaseMapMatrixHandler* handler =
      PyramidMapMatrixHandlerSelector::AllocPyramidMapMatrixHandler(
          MapVersion::LOSSLESS_MAP);

  // create pyramid map matrix
  std::unique_ptr<PyramidMapConfig> config(new PyramidMapConfig("lossy_map"));
  config->SetMapNodeSize(2, 2);
  config->resolution_num_ = 1;
  std::shared_ptr<PyramidMapMatrix> pm_matrix(new PyramidMapMatrix());
  pm_matrix->Init(*config);

  // set matrix data
  float float_data[] = {0.f, 256.f, 10.f, 10.f};
  unsigned int uint_data[] = {10, 10, 10, 10};
  pm_matrix->SetIntensityMatrix(float_data, 4, 0);
  pm_matrix->SetIntensityVarMatrix(float_data, 4, 0);
  pm_matrix->SetAltitudeMatrix(float_data, 4, 0);
  pm_matrix->SetAltitudeVarMatrix(float_data, 4, 0);
  pm_matrix->SetGroundAltitudeMatrix(float_data, 4, 0);
  pm_matrix->SetGroundCountMatrix(uint_data, 4, 0);
  pm_matrix->SetCountMatrix(uint_data, 4, 0);

  // normal case: check get/create/load
  std::shared_ptr<unsigned char> buf(new unsigned char[200]);
  unsigned int expect_size = 184;
  EXPECT_EQ(handler->GetBinarySize(pm_matrix), expect_size);
  EXPECT_EQ(handler->CreateBinary(pm_matrix, buf.get(), expect_size - 1), 0);
  EXPECT_EQ(handler->CreateBinary(pm_matrix, buf.get(), expect_size),
            expect_size);
  EXPECT_EQ(handler->LoadBinary(buf.get(), pm_matrix), expect_size);

  // check the matrix after save and load
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensitySafe(0, 1), 256.f);

  // bad case 1: no point in ground
  unsigned int uint_ground_data[] = {0, 0, 0, 0};
  pm_matrix->SetGroundCountMatrix(uint_ground_data, 4, 0);
  std::shared_ptr<unsigned char> ground_buf(new unsigned char[200]);
  expect_size = 104;
  EXPECT_EQ(handler->GetBinarySize(pm_matrix), expect_size);
  EXPECT_EQ(handler->CreateBinary(pm_matrix, ground_buf.get(), expect_size - 1),
            0);
  EXPECT_EQ(handler->CreateBinary(pm_matrix, ground_buf.get(), expect_size),
            expect_size);
  EXPECT_EQ(handler->LoadBinary(ground_buf.get(), pm_matrix), expect_size);

  // bad case 2: flags are false
  config->has_intensity_ = false;
  config->has_intensity_var_ = false;
  config->has_altitude_ = false;
  config->has_altitude_var_ = false;
  config->has_ground_altitude_ = false;
  config->has_ground_count_ = false;
  config->has_count_ = false;
  pm_matrix->Init(*config);
  expect_size = 104;
  EXPECT_EQ(handler->GetBinarySize(pm_matrix), expect_size);
  EXPECT_EQ(handler->CreateBinary(pm_matrix, buf.get(), expect_size - 1), 0);
  EXPECT_EQ(handler->CreateBinary(pm_matrix, buf.get(), expect_size),
            expect_size);
  EXPECT_EQ(handler->LoadBinary(buf.get(), pm_matrix), expect_size);

  if (handler != nullptr) {
    delete handler;
    handler = nullptr;
  }
}

TEST_F(PyramidMapMatrixHandlerTestSuite, PyramidLossyMapMatrixHandler) {
  // create handler
  BaseMapMatrixHandler* handler =
      PyramidMapMatrixHandlerSelector::AllocPyramidMapMatrixHandler(
          MapVersion::PYRAMID_LOSSY_MAP);

  // create pyramid map matrix
  std::unique_ptr<PyramidMapConfig> config(
      new PyramidMapConfig("pyramid_lossy_map"));
  config->SetMapNodeSize(2, 2);
  config->resolution_num_ = 1;
  std::shared_ptr<PyramidMapMatrix> pm_matrix(new PyramidMapMatrix());
  pm_matrix->Init(*config);

  // set matrix data
  float float_data[] = {-1.f, 256.f, 10.f, 10.f};
  unsigned int uint_data[] = {10, 10, 10, 0};
  pm_matrix->SetIntensityMatrix(float_data, 4, 0);
  pm_matrix->SetIntensityVarMatrix(float_data, 4, 0);
  pm_matrix->SetAltitudeMatrix(float_data, 4, 0);
  pm_matrix->SetAltitudeVarMatrix(float_data, 4, 0);
  pm_matrix->SetGroundAltitudeMatrix(float_data, 4, 0);
  pm_matrix->SetGroundCountMatrix(uint_data, 4, 0);
  pm_matrix->SetCountMatrix(uint_data, 4, 0);

  // normal case: check get/create/load
  std::shared_ptr<unsigned char> buf(new unsigned char[100]);
  unsigned int expect_size = 72;
  EXPECT_EQ(handler->GetBinarySize(pm_matrix), expect_size);
  EXPECT_EQ(handler->CreateBinary(pm_matrix, buf.get(), expect_size - 1), 0);
  EXPECT_EQ(handler->CreateBinary(pm_matrix, buf.get(), expect_size),
            expect_size);
  EXPECT_EQ(handler->LoadBinary(buf.get(), pm_matrix), expect_size);

  // check the matrix after save and load
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensitySafe(0, 1), 255.f);

  // bad case 1: flags are false
  config->has_intensity_ = false;
  config->has_intensity_var_ = false;
  config->has_altitude_ = false;
  config->has_altitude_var_ = false;
  config->has_ground_altitude_ = false;
  config->has_ground_count_ = false;
  config->has_count_ = false;
  pm_matrix->Init(*config);
  expect_size = 28;
  EXPECT_EQ(handler->GetBinarySize(pm_matrix), expect_size);
  EXPECT_EQ(handler->CreateBinary(pm_matrix, buf.get(), expect_size - 1), 0);
  EXPECT_EQ(handler->CreateBinary(pm_matrix, buf.get(), expect_size),
            expect_size);
  EXPECT_EQ(handler->LoadBinary(buf.get(), pm_matrix), expect_size);

  if (handler != nullptr) {
    delete handler;
    handler = nullptr;
  }
}

TEST_F(PyramidMapMatrixHandlerTestSuite, PyramidLosslessMapMatrixHandler) {
  // create handler
  BaseMapMatrixHandler* handler =
      PyramidMapMatrixHandlerSelector::AllocPyramidMapMatrixHandler(
          MapVersion::PYRAMID_LOSSLESS_MAP);

  // create pyramid map matrix
  std::unique_ptr<PyramidMapConfig> config(
      new PyramidMapConfig("pyramid_lossless_map"));
  config->SetMapNodeSize(2, 2);
  config->resolution_num_ = 1;
  std::shared_ptr<PyramidMapMatrix> pm_matrix(new PyramidMapMatrix());
  pm_matrix->Init(*config);

  // set matrix data
  float float_data[] = {0.f, 256.f, 10.f, 10.f};
  unsigned int uint_data[] = {10, 10, 10, 10};
  pm_matrix->SetIntensityMatrix(float_data, 4, 0);
  pm_matrix->SetIntensityVarMatrix(float_data, 4, 0);
  pm_matrix->SetAltitudeMatrix(float_data, 4, 0);
  pm_matrix->SetAltitudeVarMatrix(float_data, 4, 0);
  pm_matrix->SetGroundAltitudeMatrix(float_data, 4, 0);
  pm_matrix->SetGroundCountMatrix(uint_data, 4, 0);
  pm_matrix->SetCountMatrix(uint_data, 4, 0);

  // normal case: check get/create/load
  std::shared_ptr<unsigned char> buf(new unsigned char[200]);
  unsigned int expect_size = 132;
  EXPECT_EQ(handler->GetBinarySize(pm_matrix), expect_size);
  EXPECT_EQ(handler->CreateBinary(pm_matrix, buf.get(), expect_size - 1), 0);
  EXPECT_EQ(handler->CreateBinary(pm_matrix, buf.get(), expect_size),
            expect_size);
  EXPECT_EQ(handler->LoadBinary(buf.get(), pm_matrix), expect_size);

  // check the matrix after save and load
  EXPECT_FLOAT_EQ(*pm_matrix->GetIntensitySafe(0, 1), 256.f);

  // bad case 1: flags are false
  config->has_intensity_ = false;
  config->has_intensity_var_ = false;
  config->has_altitude_ = false;
  config->has_altitude_var_ = false;
  config->has_ground_altitude_ = false;
  config->has_ground_count_ = false;
  config->has_count_ = false;
  pm_matrix->Init(*config);
  expect_size = 20;
  EXPECT_EQ(handler->GetBinarySize(pm_matrix), expect_size);
  EXPECT_EQ(handler->CreateBinary(pm_matrix, buf.get(), expect_size - 1), 0);
  EXPECT_EQ(handler->CreateBinary(pm_matrix, buf.get(), expect_size),
            expect_size);
  EXPECT_EQ(handler->LoadBinary(buf.get(), pm_matrix), expect_size);

  if (handler != nullptr) {
    delete handler;
    handler = nullptr;
  }
}

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
