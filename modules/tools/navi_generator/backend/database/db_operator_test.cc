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
#include "modules/tools/navi_generator/backend/database/db_operator.h"

#include <memory>

#include "gtest/gtest.h"
#include "modules/common/log.h"

namespace apollo {
namespace navi_generator {
namespace util {

class DBOperatorTest : public testing::Test {
 public:
  virtual void SetUp() { db_operator_.reset(new DBOperator()); }

 protected:
  std::unique_ptr<DBOperator> db_operator_;
};

#define EXPECT_EQ_ARRAY(len, x, y, msg)                                  \
  for (std::size_t j = 0; j < len; ++j) {                                \
    EXPECT_EQ(x[j], y[j]) << "" #x << " != " #y << " byte " << j << ": " \
                          << msg;                                        \
  }

TEST_F(DBOperatorTest, InitDB) {
  EXPECT_TRUE(db_operator_->InitDatabase());

  std::uint64_t start_way_id = 4294967296;
  std::uint8_t way_counts = 5;

  EXPECT_TRUE(db_operator_->DeleteWay(start_way_id));

  // query Way which is not existing
  Way way;
  EXPECT_FALSE(db_operator_->QueryWayWithWayId(start_way_id, &way));

  // save and query Way which has no pre and next way
  way = {start_way_id, 0, 0};
  Way way_comp;
  EXPECT_TRUE(db_operator_->DeleteWay(way.way_id));
  EXPECT_TRUE(db_operator_->SaveWay(way));
  EXPECT_TRUE(db_operator_->QueryWayWithWayId(way.way_id, &way_comp));
  EXPECT_EQ(way.way_id, way_comp.way_id);
  EXPECT_EQ(way.pre_way_id, way_comp.pre_way_id);
  EXPECT_EQ(way.next_way_id, way_comp.next_way_id);

  // save and query Way which has pre and next way
  EXPECT_TRUE(db_operator_->DeleteWay(start_way_id));
  way = {start_way_id, start_way_id - 1, start_way_id + 1};
  EXPECT_TRUE(db_operator_->SaveWay(way));
  EXPECT_TRUE(db_operator_->QueryWayWithWayId(way.way_id, &way_comp));
  EXPECT_EQ(way.way_id, way_comp.way_id);
  EXPECT_EQ(way.pre_way_id, way_comp.pre_way_id);
  EXPECT_EQ(way.next_way_id, way_comp.next_way_id);

  // save and query WayNodes
  WayNodes way_nodes;
  way_nodes.way_id = start_way_id;
  std::vector<Node> nodes;
  for (std::size_t i = 0; i < 5; ++i) {
    Node node = {i, i, "ABCDEFGH" + std::to_string(i)};
    nodes.emplace_back(node);
  }
  way_nodes.nodes = nodes;
  EXPECT_TRUE(db_operator_->SaveWayNodes(way_nodes));
  WayNodes way_nodes_comp;
  EXPECT_TRUE(
      db_operator_->QueryWayNodesWithWayId(way_nodes.way_id, &way_nodes_comp));
  EXPECT_EQ(way_nodes_comp.way_id, way_nodes.way_id);
  for (std::size_t i = 0; i < 5; ++i) {
    EXPECT_EQ(way_nodes_comp.nodes[i].node_index, i);
    EXPECT_EQ(way_nodes_comp.nodes[i].data_line_number, i);
    EXPECT_EQ(way_nodes_comp.nodes[i].node_value,
              "ABCDEFGH" + std::to_string(i));
  }

  // save and query WayData
  std::string data = "abcdefgh";
  std::vector<unsigned char> raw_data;
  raw_data.assign(data.begin(), data.end());
  WayData way_data = {start_way_id, raw_data, 3, 0};
  WayData way_data_comp;
  EXPECT_TRUE(db_operator_->SaveWayData(way_data));
  EXPECT_TRUE(
      db_operator_->QueryWayDataWithWayId(way_data.way_id, &way_data_comp));
  EXPECT_EQ(way_data.way_id, way_data_comp.way_id);
  EXPECT_EQ_ARRAY(way_data.raw_data.size(), way_data.raw_data.data(),
                  way_data_comp.raw_data.data(), 0);
  EXPECT_EQ(way_data.navi_number, way_data_comp.navi_number);
  EXPECT_EQ(way_data.navi_table_id, way_data_comp.navi_table_id);

  // save and query NaviData
  NaviInfo navi_info;
  navi_info.way_id = start_way_id;
  NaviData navi_data;
  std::string str_data;
  for (std::size_t i = 0; i < 3; ++i) {
    navi_data.navi_index = i;
    str_data = "abcdefgh" + std::to_string(i);
    navi_data.data.assign(str_data.begin(), str_data.end());
    navi_info.navi_data.emplace_back(navi_data);
  }
  EXPECT_TRUE(db_operator_->SaveNaviData(navi_info));
  std::vector<NaviData> navi_data_comp;
  EXPECT_TRUE(
      db_operator_->QueryNaviDataWithWayId(start_way_id, &navi_data_comp));
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(navi_data_comp[i].navi_index, i);
    str_data = "abcdefgh" + std::to_string(i);
    std::vector<unsigned char> data;
    data.assign(str_data.begin(), str_data.end());
    EXPECT_EQ_ARRAY(navi_data_comp[i].data.size(),
                    navi_data_comp[i].data.data(), data.data(), 0);
  }
  NaviData navi_comp;
  EXPECT_TRUE(
      db_operator_->QueryNaviDataWithWayId(start_way_id, 0, &navi_comp));
  EXPECT_EQ(navi_comp.navi_index, 0);
  str_data = "abcdefgh" + std::to_string(0);
  std::vector<unsigned char> navi;
  navi.assign(str_data.begin(), str_data.end());
  EXPECT_EQ_ARRAY(navi_comp.data.size(), navi_comp.data.data(), navi.data(), 0);

  // save more data for testing query route
  for (std::uint64_t i = start_way_id + 1; i < start_way_id + way_counts; ++i) {
    way.way_id = i;
    way.pre_way_id = i - 1;
    way.next_way_id = i + 1;
    EXPECT_TRUE(db_operator_->SaveWay(way));
    EXPECT_TRUE(db_operator_->QueryWayWithWayId(way.way_id, &way_comp));
    EXPECT_EQ(way.way_id, way_comp.way_id);
    EXPECT_EQ(way.pre_way_id, way_comp.pre_way_id);
    EXPECT_EQ(way.next_way_id, way_comp.next_way_id);
  }

  std::uint64_t id = start_way_id;
  std::vector<apollo::navi_generator::util::Way> route;
  EXPECT_TRUE(db_operator_->QueryRouteWithStartEndWay(start_way_id,
                                                      start_way_id, &route));
  for (auto& way : route) {
    EXPECT_EQ(way.way_id, id);
    ++id;
  }

  id = start_way_id;
  route.clear();
  EXPECT_TRUE(db_operator_->QueryRouteWithStartEndWay(
      start_way_id, start_way_id + way_counts - 1, &route));
  for (auto& way : route) {
    EXPECT_EQ(way.way_id, id);
    ++id;
  }

  id = start_way_id + way_counts - 1;
  route.clear();
  EXPECT_TRUE(db_operator_->QueryRouteWithStartEndWay(
      start_way_id + way_counts - 1, start_way_id, &route));
  for (auto& way : route) {
    EXPECT_EQ(way.way_id, id);
    --id;
  }

  route.clear();
  EXPECT_FALSE(db_operator_->QueryRouteWithStartEndWay(
      start_way_id, start_way_id + way_counts, &route));

  route.clear();
  EXPECT_FALSE(db_operator_->QueryRouteWithStartEndWay(
      start_way_id - 1, start_way_id + way_counts - 1, &route));

  for (std::uint64_t i = start_way_id; i < start_way_id + way_counts; ++i) {
    EXPECT_TRUE(db_operator_->DeleteWay(i));
    EXPECT_FALSE(db_operator_->QueryWayWithWayId(i, &way));
  }
}

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
