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

#ifndef MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_DB_OPERATOR_H_
#define MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_DB_OPERATOR_H_

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

enum TableNames {
  kTableSpeedLimit = 0,
  kTableWay,
  kTableWayNodes,
  kTableWayData,
  kTableNaviData,
  kMaxNumberOfTables,
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

 private:
  bool OpenDatabase();
  void CloseDatabase();
  bool FillTableSpeedLimit();
};

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo

#endif  // MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_DB_OPERATOR_H_
