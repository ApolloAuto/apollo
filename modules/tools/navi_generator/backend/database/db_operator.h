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
 * @brief This file provides the declaration of the class "DBOperator"
 */

#ifndef MODULES_TOOLS_NAVI_GENERATOR_BACKEND_DATABASE_DB_OPERATOR_H_
#define MODULES_TOOLS_NAVI_GENERATOR_BACKEND_DATABASE_DB_OPERATOR_H_

#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "modules/tools/navi_generator/backend/database/db_helper.h"
/**
 * @namespace apollo::navi_generator::util
 * @brief apollo::navi_generator::util
 */
namespace apollo {
namespace navi_generator {
namespace util {

struct SpeedLimit {
  std::uint8_t id;
  std::uint8_t speed;
};

struct Way {
  std::uint64_t way_id;       // starting from 1
  std::uint64_t pre_way_id;   // starting from 1
  std::uint64_t next_way_id;  // starting from 1
  std::uint8_t speed_min;
  std::uint8_t speed_max;
};

struct Node {
  std::uint64_t node_index;  // starting from 0
  std::uint64_t
      data_line_number;  // starting from 1. the line number of the binary file
                         // which corresponding to node_index.
  std::string node_value;
};

struct WayNodes {
  std::uint64_t way_id;  // starting from 1
  std::vector<Node> nodes;
};

struct WayData {
  std::uint64_t way_id;                 // starting from 1
  std::vector<unsigned char> raw_data;  // the data of the raw bag file
  std::uint8_t navi_number;
  std::uint64_t navi_table_id;
};

struct NaviData {
  std::uint8_t navi_index;  // starting from 0
  std::vector<unsigned char> data;
};

struct NaviInfo {
  std::uint64_t way_id;  // starting from 1
  std::vector<NaviData> navi_data;
};

struct NaviInfoWithPos {
  std::uint64_t way_id;  // starting from 1
  Node node;
  std::vector<NaviData> navi_data;
};

enum TableNames {
  kTableSpeedLimit = 0,
  kTableWay,
  kTableWayNodes,
  kTableWayData,
  kTableNaviData,
  kMaxNumberOfTables,
};

enum Orientation {
  kOrientationForward = 0,
  kOrientationBackward,
};

class DBOperatorBase {
 public:
  DBOperatorBase() = default;
  ~DBOperatorBase() = default;

 public:
  virtual bool CreateTable(const TableNames table_name);
  virtual bool IsTableExisting(const TableNames table_name);

 protected:
  apollo::navi_generator::database::SQLite sqlite_;
};

class DBOperator : public DBOperatorBase {
 public:
  DBOperator();
  ~DBOperator();

 public:
  /**
   * @brief Initialize the database, create database tables, and fill the fixed
   * data for some tables.
   * @return  Return true for success.
   */
  bool InitDatabase();
  bool SaveWay(const Way& way);
  bool SaveWayNodes(const WayNodes& way_nodes);
  bool SaveWayData(const WayData& way_data);
  bool SaveNaviData(const NaviInfo& navi_info);
  bool QueryWayWithWayId(const std::uint64_t way_id, Way* const way);
  bool QueryWayDataWithWayId(const std::uint64_t way_id,
                             WayData* const way_data);
  bool QueryNaviDataWithWayId(const std::uint64_t way_id,
                              std::vector<NaviData>* const navi_data);
  bool QueryNaviDataWithWayId(const std::uint64_t way_id,
                              const std::uint8_t navi_index,
                              NaviData* const navi_data);
  bool QueryWayNodesWithWayId(const std::uint64_t way_id,
                              WayNodes* const way_nodes);
  bool UpdateWay(const std::uint64_t way_id, const Way& way);
  bool UpdateWaySpeedLimit(const std::uint64_t way_id,
                           const std::uint8_t speed_min,
                           const std::uint8_t speed_max);
  bool UpdateWayNodes(const std::uint64_t way_id, const WayNodes& way_nodes);
  bool UpdateWayData(const std::uint64_t way_id, const WayData& way_data);
  bool UpdateNaviData(const std::uint64_t way_id, const NaviInfo& navi_info);
  bool DeleteWay(const std::uint64_t way_id);
  bool CreateNewWayId(std::uint64_t* const way_id);
  bool GetNaviTableId(std::uint64_t* const navi_table_id);
  bool QueryNaviWithPos(const std::string& node_value,
                        std::vector<NaviInfoWithPos>* const navi_info);
  bool QueryRouteWithStartEndWay(const std::uint64_t start_way_id,
                                 const std::uint64_t end_way_id,
                                 std::vector<Way>* const route);

 private:
  bool OpenDatabase();
  void CloseDatabase();
  bool FillTableSpeedLimit();
  bool DeleteWayNodes(const std::uint64_t way_id);
  bool DeleteWayData(const std::uint64_t way_id);
  bool DeleteNaviData(const std::uint64_t way_id);
  bool FindNaviWithPos(const std::string& node_value,
                       std::vector<NaviInfoWithPos>* const navi_info);
  bool FindRoute(const std::uint64_t start_way_id,
                 const std::uint64_t end_way_id, const Orientation orientation,
                 std::vector<Way>* const route);
};

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo

#endif  // MODULES_TOOLS_NAVI_GENERATOR_BACKEND_DATABASE_DB_OPERATOR_H_
