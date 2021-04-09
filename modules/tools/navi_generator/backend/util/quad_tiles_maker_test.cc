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

/**
 * @file
 * @brief This file provides several unit tests for the class
 * "QuadTilesMaker".
 */
#include "modules/tools/navi_generator/backend/util/quad_tiles_maker.h"

#include <memory>

#include "gtest/gtest.h"

namespace apollo {
namespace navi_generator {
namespace util {

class QuadTilesMakerTest : public testing::Test {
 public:
  virtual void SetUp() { quad_tiles_maker_.reset(new QuadTilesMaker()); }

 protected:
  std::unique_ptr<QuadTilesMaker> quad_tiles_maker_;
};

TEST_F(QuadTilesMakerTest, GoodData) {
  QuadTile quad_tile;
  std::string id_string;
  std::uint32_t id_uint32;

  EXPECT_TRUE(quad_tiles_maker_->MakeQuadTile(-0.000000001, -0.000000001, 0.0,
                                              16, &quad_tile));
  EXPECT_TRUE(quad_tiles_maker_->IdAsString(16, &quad_tile, &id_string));
  EXPECT_EQ(id_string, "cbbbbbbbbbbbbbbb");
  EXPECT_TRUE(quad_tiles_maker_->IdAsUint32(16, &quad_tile, &id_uint32));
  EXPECT_EQ(id_uint32, 0x95555555);
}

TEST_F(QuadTilesMakerTest, InvalidData) {
  QuadTile quad_tile;
  std::string id_string;
  std::uint32_t id_uint32;
  // invalid level (<1)
  EXPECT_FALSE(
      quad_tiles_maker_->MakeQuadTile(30.0, 150.0, 10.0, 0, &quad_tile));
  // invalid level (>32)
  EXPECT_FALSE(
      quad_tiles_maker_->MakeQuadTile(30.0, 150.0, 10.0, 33, &quad_tile));
  // invalid lat (<-90.0)
  EXPECT_FALSE(
      quad_tiles_maker_->MakeQuadTile(-91.0, 150.0, 10.0, 16, &quad_tile));
  // invalid lat (>90.0)
  EXPECT_FALSE(
      quad_tiles_maker_->MakeQuadTile(91.0, 150.0, 10.0, 16, &quad_tile));
  // invalid lon (<=-180.0)
  EXPECT_FALSE(
      quad_tiles_maker_->MakeQuadTile(30.0, -180.0, 10.0, 16, &quad_tile));
  // invalid lon (>180.0)
  EXPECT_FALSE(
      quad_tiles_maker_->MakeQuadTile(30.0, 181.0, 10.0, 16, &quad_tile));

  // good data
  EXPECT_TRUE(
      quad_tiles_maker_->MakeQuadTile(30.0, 150.0, 10.0, 8, &quad_tile));
  // invalid level (<1)
  EXPECT_FALSE(quad_tiles_maker_->IdAsString(0, &quad_tile, &id_string));
  // invalid level (>quad_tile(id))
  EXPECT_FALSE(quad_tiles_maker_->IdAsString(16, &quad_tile, &id_string));
  // invalid level (<1)
  EXPECT_FALSE(quad_tiles_maker_->IdAsUint32(0, &quad_tile, &id_uint32));
  // invalid level (>16)
  EXPECT_FALSE(quad_tiles_maker_->IdAsUint32(17, &quad_tile, &id_uint32));
  // invalid level (>quad_tile(id))
  EXPECT_FALSE(quad_tiles_maker_->IdAsUint32(16, &quad_tile, &id_uint32));
}

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
