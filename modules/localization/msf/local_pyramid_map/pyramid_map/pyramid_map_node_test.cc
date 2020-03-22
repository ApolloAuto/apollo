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
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_node.h"

#include "gtest/gtest.h"

#include "modules/localization/msf/local_pyramid_map/base_map/base_map_node.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_node_config.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_config.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_node_config.h"

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

class PyramidMapNodeTestSuite : public ::testing::Test {
 protected:
  PyramidMapNodeTestSuite() {}
  virtual ~PyramidMapNodeTestSuite() {}
  virtual void SetUp() {}
  virtual void TearDown() {}
};

TEST_F(PyramidMapNodeTestSuite, init) {
  // create index
  MapNodeIndex index;
  index.resolution_id_ = 0;
  index.zone_id_ = 50;
  index.m_ = 0;
  index.n_ = 0;

  // init config
  PyramidMapConfig* config = new PyramidMapConfig("lossy_full_alt");
  config->map_version_ = "pyramid_lossy_map";
  config->SetMapNodeSize(2, 2);
  config->resolution_num_ = 2;

  // create map node
  PyramidMapNode* node = new PyramidMapNode();
  node->Init(config);
  node->SetMapNodeIndex(index);

  node->Init(config, index);
  config->map_version_ = "lossy_full_alt";
  node->Init(config, index);

  if (node != nullptr) {
    delete node;
    node = nullptr;
  }
  if (config != nullptr) {
    delete config;
    config = nullptr;
  }
}

TEST_F(PyramidMapNodeTestSuite, pyramid_map_node_function) {
  // create index
  MapNodeIndex index;
  index.resolution_id_ = 0;
  index.zone_id_ = 50;
  index.m_ = 0;
  index.n_ = 0;

  // init config
  PyramidMapConfig* config = new PyramidMapConfig("lossy_full_alt");
  config->SetMapNodeSize(2, 2);
  config->resolution_num_ = 2;

  // create map node
  PyramidMapNode* node = new PyramidMapNode();
  node->Init(config);
  node->SetMapNodeIndex(index);

  // check global coordinate to local coordinate transformation
  double data[] = {0.125, 0.125, 1.0};
  unsigned int x = 0;
  unsigned int y = 0;
  Eigen::Vector2d vector_2d_data(data);
  node->GetCoordinate(vector_2d_data, 0, &x, &y);
  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 1);

  // good case: set and get method
  Eigen::Vector3d vector_3d_data(data);
  EXPECT_TRUE(node->AddValueIfInBound(vector_3d_data, 10, 0));
  EXPECT_FLOAT_EQ(node->GetIntensitySafe(vector_3d_data, 0), 10.f);
  EXPECT_FLOAT_EQ(node->GetIntensityVarSafe(vector_3d_data, 0), 0.f);
  EXPECT_FLOAT_EQ(node->GetAltitudeSafe(vector_3d_data, 0), 1.f);
  EXPECT_FLOAT_EQ(node->GetAltitudeVarSafe(vector_3d_data, 0), 0.f);
  EXPECT_EQ(node->GetCountSafe(vector_3d_data, 0), 1);
  EXPECT_EQ(node->GetGroundCountSafe(vector_3d_data, 0), 0);
  EXPECT_FLOAT_EQ(node->GetGroundAltitudeSafe(vector_3d_data, 0), 0.f);

  EXPECT_FLOAT_EQ(node->GetIntensity(vector_3d_data, 0), 10.f);
  EXPECT_FLOAT_EQ(node->GetIntensityVar(vector_3d_data, 0), 0.f);
  EXPECT_FLOAT_EQ(node->GetAltitude(vector_3d_data, 0), 1.f);
  EXPECT_FLOAT_EQ(node->GetAltitudeVar(vector_3d_data, 0), 0.f);
  EXPECT_EQ(node->GetCount(vector_3d_data, 0), 1);
  EXPECT_EQ(node->GetGroundCount(vector_3d_data, 0), 0);
  EXPECT_FLOAT_EQ(node->GetGroundAltitude(vector_3d_data, 0), 0.f);

  // bottom up
  node->BottomUpSafe();
  EXPECT_FLOAT_EQ(node->GetIntensitySafe(vector_3d_data, 1), 10.f);
  EXPECT_FLOAT_EQ(node->GetIntensityVarSafe(vector_3d_data, 1), 0.f);
  EXPECT_FLOAT_EQ(node->GetAltitudeSafe(vector_3d_data, 1), 1.f);
  EXPECT_FLOAT_EQ(node->GetAltitudeVarSafe(vector_3d_data, 1), 0.f);
  EXPECT_EQ(node->GetCountSafe(vector_3d_data, 1), 1);
  EXPECT_EQ(node->GetGroundCountSafe(vector_3d_data, 1), 0);

  // bad case 1: get coordinate failed
  vector_3d_data[0] = 10.0;
  vector_3d_data[1] = 10.0;
  EXPECT_FLOAT_EQ(node->GetIntensitySafe(vector_3d_data, 0), 0.f);
  EXPECT_FLOAT_EQ(node->GetIntensityVarSafe(vector_3d_data, 0), 0.f);
  EXPECT_FLOAT_EQ(node->GetAltitudeSafe(vector_3d_data, 0), 0.f);
  EXPECT_FLOAT_EQ(node->GetAltitudeVarSafe(vector_3d_data, 0), 0.f);
  EXPECT_EQ(node->GetCountSafe(vector_3d_data, 0), 0);
  EXPECT_EQ(node->GetGroundCountSafe(vector_3d_data, 0), 0);
  EXPECT_FLOAT_EQ(node->GetGroundAltitudeSafe(vector_3d_data, 0), 0.f);

  EXPECT_FLOAT_EQ(node->GetIntensity(vector_3d_data, 0), 0.f);
  EXPECT_FLOAT_EQ(node->GetIntensityVar(vector_3d_data, 0), 0.f);
  EXPECT_FLOAT_EQ(node->GetAltitude(vector_3d_data, 0), 0.f);
  EXPECT_FLOAT_EQ(node->GetAltitudeVar(vector_3d_data, 0), 0.f);
  EXPECT_EQ(node->GetCount(vector_3d_data, 0), 0);
  EXPECT_EQ(node->GetGroundCount(vector_3d_data, 0), 0);
  EXPECT_FLOAT_EQ(node->GetGroundAltitude(vector_3d_data, 0), 0.f);

  // check branch when reset
  node->ResetMapNode();
  vector_3d_data[0] = 0.125;
  vector_3d_data[1] = 0.125;
  EXPECT_FLOAT_EQ(node->GetIntensitySafe(vector_3d_data, 0), 0.f);
  EXPECT_FLOAT_EQ(node->GetIntensityVarSafe(vector_3d_data, 0), 0.f);
  EXPECT_FLOAT_EQ(node->GetAltitudeSafe(vector_3d_data, 0), 0.f);
  EXPECT_FLOAT_EQ(node->GetAltitudeVarSafe(vector_3d_data, 0), 0.f);
  EXPECT_EQ(node->GetCountSafe(vector_3d_data, 0), 0);
  EXPECT_EQ(node->GetGroundCountSafe(vector_3d_data, 0), 0);
  EXPECT_FLOAT_EQ(node->GetGroundAltitudeSafe(vector_3d_data, 0), 0.f);

  EXPECT_FLOAT_EQ(node->GetIntensity(vector_3d_data, 0), 0.f);
  EXPECT_FLOAT_EQ(node->GetIntensityVar(vector_3d_data, 0), 0.f);
  EXPECT_FLOAT_EQ(node->GetAltitude(vector_3d_data, 0), 0.f);
  EXPECT_FLOAT_EQ(node->GetAltitudeVar(vector_3d_data, 0), 0.f);
  EXPECT_EQ(node->GetCount(vector_3d_data, 0), 0);
  EXPECT_EQ(node->GetGroundCount(vector_3d_data, 0), 0);
  EXPECT_FLOAT_EQ(node->GetGroundAltitude(vector_3d_data, 0), 0.f);

  if (node != nullptr) {
    delete node;
    node = nullptr;
  }
  if (config != nullptr) {
    delete config;
    config = nullptr;
  }
}

TEST_F(PyramidMapNodeTestSuite, test_get_set_bad_case) {
  // create index
  MapNodeIndex index;
  index.resolution_id_ = 0;
  index.zone_id_ = 50;
  index.m_ = 0;
  index.n_ = 0;

  // init config
  PyramidMapConfig* config = new PyramidMapConfig("lossy_full_alt");
  config->SetMapNodeSize(2, 2);
  config->resolution_num_ = 1;
  config->has_intensity_ = false;
  config->has_intensity_var_ = false;
  config->has_altitude_ = false;
  config->has_altitude_var_ = false;
  config->has_ground_altitude_ = false;
  config->has_ground_count_ = false;
  config->has_count_ = false;

  // create map node
  PyramidMapNode* node = new PyramidMapNode();
  node->Init(config);
  node->SetMapNodeIndex(index);

  // check global coordinate to local coordinate transformation
  double data[] = {0.125, 0.125, 1.0};
  Eigen::Vector3d vector_3d_data(data);
  EXPECT_FLOAT_EQ(node->GetIntensitySafe(vector_3d_data, 0), 0.f);
  EXPECT_FLOAT_EQ(node->GetIntensityVarSafe(vector_3d_data, 0), 0.f);
  EXPECT_FLOAT_EQ(node->GetAltitudeSafe(vector_3d_data, 0), 0.f);
  EXPECT_FLOAT_EQ(node->GetAltitudeVarSafe(vector_3d_data, 0), 0.f);
  EXPECT_EQ(node->GetCountSafe(vector_3d_data, 0), 0);
  EXPECT_EQ(node->GetGroundCountSafe(vector_3d_data, 0), 0);
  EXPECT_FLOAT_EQ(node->GetGroundAltitudeSafe(vector_3d_data, 0), 0.f);
}

TEST_F(PyramidMapNodeTestSuite, test_base_map_node_save_load) {
  // create index
  MapNodeIndex index;
  index.resolution_id_ = 0;
  index.zone_id_ = 50;
  index.m_ = 0;
  index.n_ = 1;

  // init config
  PyramidMapConfig* config = new PyramidMapConfig("lossy_full_alt");
  config->SetMapNodeSize(2, 2);
  config->resolution_num_ = 1;
  config->map_folder_path_ = "wrong_path";

  // create map node
  PyramidMapNode* node = new PyramidMapNode();
  node->Init(config);
  node->SetMapNodeIndex(index);
  EXPECT_EQ(node->GetMapNodeIndex().m_, 0);
  EXPECT_EQ(node->GetMapNodeIndex().n_, 1);

  // check Load
  EXPECT_FALSE(node->Load());
  EXPECT_FALSE(node->Load("map.bin"));

  if (node != nullptr) {
    delete node;
    node = nullptr;
  }
  if (config != nullptr) {
    delete config;
    config = nullptr;
  }
}

TEST_F(PyramidMapNodeTestSuite, test_base_map_node) {
  // create index
  MapNodeIndex index;
  index.resolution_id_ = 0;
  index.zone_id_ = 50;
  index.m_ = 0;
  index.n_ = 1;

  // init config
  PyramidMapConfig* config = new PyramidMapConfig("lossy_full_alt");
  config->SetMapNodeSize(2, 2);
  config->resolution_num_ = 1;

  // create map node
  PyramidMapNode* node = new PyramidMapNode();
  node->Init(config);
  node->SetMapNodeIndex(index);
  EXPECT_EQ(node->GetMapNodeIndex().m_, 0);
  EXPECT_EQ(node->GetMapNodeIndex().n_, 1);

  // add value
  double data[] = {0.125, 0.125, 1.0};
  Eigen::Vector3d vector_3d_data(data);
  EXPECT_FALSE(node->AddValueIfInBound(vector_3d_data, 10, 0));
  vector_3d_data[0] = 0.375;
  EXPECT_TRUE(node->AddValueIfInBound(vector_3d_data, 10, 0));

  // Save and Load
  // Load -> load_binary -> get_header_binary_size -> load_header_binary ->
  // load_body_binary
  EXPECT_TRUE(node->Save());
  EXPECT_TRUE(node->Load());

  // check bad case
  EXPECT_TRUE(node->GetIsReady());

  // check calculate left top corner
  Eigen::Vector2d left_top_corner = node->ComputeLeftTopCorner(*config, index);
  EXPECT_FLOAT_EQ(left_top_corner[0], 0.25);
  EXPECT_FLOAT_EQ(left_top_corner[1], 0.0);

  // check Get map cell matrix
  const BaseMapMatrix& map_matrix_const = node->GetMapCellMatrix();
  const PyramidMapMatrix& pyramid_matrix_const =
      dynamic_cast<const PyramidMapMatrix&>(map_matrix_const);
  EXPECT_FLOAT_EQ(*pyramid_matrix_const.GetIntensitySafe(1, 1, 0), 10.f);
  BaseMapMatrix& map_matrix = node->GetMapCellMatrix();
  PyramidMapMatrix& pyramid_matrix =
      dynamic_cast<PyramidMapMatrix&>(map_matrix);
  EXPECT_FLOAT_EQ(*pyramid_matrix.GetIntensitySafe(1, 1, 0), 10.f);

  // check Get the map config
  const BaseMapConfig& bm_config = node->GetMapConfig();
  const PyramidMapConfig& pm_config =
      dynamic_cast<const PyramidMapConfig&>(bm_config);
  EXPECT_EQ(pm_config.resolution_num_, 1);

  // check Get the map node config
  const BaseMapNodeConfig& bmn_config = node->GetMapNodeConfig();
  EXPECT_EQ(bmn_config.map_version_, MapVersion::LOSSY_FULL_ALT_MAP);

  // check variables
  EXPECT_FALSE(node->GetIsReserved());
  EXPECT_FALSE(node->GetIsChanged());
  node->SetIsReserved(true);
  node->SetIsChanged(true);
  node->ResetMapNode();
  EXPECT_FALSE(node->GetIsReserved());
  EXPECT_FALSE(node->GetIsChanged());

  // check resolution
  EXPECT_FLOAT_EQ(node->GetMapResolution(), 0.125f);

  // add vector
  std::vector<Eigen::Vector3d> coordinates;
  vector_3d_data[0] = 0.375;
  vector_3d_data[0] = 0.125;
  coordinates.push_back(vector_3d_data);
  std::vector<unsigned char> intensities;
  intensities.push_back(0);
  node->AddValueIfInBound(coordinates, intensities, 0);
  intensities.push_back(1);
  node->AddValueIfInBound(coordinates, intensities, 0);

  if (node != nullptr) {
    delete node;
    node = nullptr;
  }
  if (config != nullptr) {
    delete config;
    config = nullptr;
  }
}

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
