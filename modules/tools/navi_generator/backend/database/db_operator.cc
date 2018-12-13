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
 * @brief This file provides the implementation of the class
 * "DBOperator".
 */
#include "modules/tools/navi_generator/backend/database/db_operator.h"

#include <algorithm>

#include "modules/common/log.h"

namespace apollo {
namespace navi_generator {
namespace util {

using apollo::navi_generator::database::SQLiteCommand;
using apollo::navi_generator::database::SQLiteDataReader;

namespace {
const char database_name[] = "navi.sqlite";
// The table names should be the same with the names in the database
const std::array<std::string, 5> kTableNames = {
    "speed_limit", "way", "way_nodes", "way_data", "navi_data"};
constexpr std::uint64_t kMaxRowNumberOfDBTable = 10000;

void DeleteSQLiteDataReader(SQLiteDataReader* reader) { delete reader; }
}  // namespace

bool DBOperatorBase::IsTableExisting(const TableNames table_name) {
  if (table_name >= TableNames::kMaxNumberOfTables) {
    AERROR << "The index of the table is not less than the total number of "
              "tables.";
    return false;
  }
  std::string sql =
      "SELECT count(*)  FROM sqlite_master WHERE type='table' AND name LIKE '" +
      kTableNames.at(table_name) + "';";
  SQLiteDataReader* reader = nullptr;
  if (!sqlite_.ExcuteQuery(sql, &reader)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    return false;
  }
  int count = 0;
  while (reader->Read()) {
    count = reader->GetIntValue(0);
  }

  DeleteSQLiteDataReader(reader);

  return (count > 0 ? true : false);
}

bool DBOperatorBase::CreateTable(const TableNames table_name) {
  std::string sql;
  switch (table_name) {
    case TableNames::kTableSpeedLimit:
      sql =
          "CREATE TABLE [speed_limit] ([id] INTEGER,[speed] "
          "INTEGER,PRIMARY KEY(id));";
      break;
    case TableNames::kTableWay:
      sql =
          "CREATE TABLE [way] ([way_id] TEXT,[pre_way_id] TEXT,[next_way_id] "
          "TEXT,[speed_min] INTEGER REFERENCES speed_limit(id) ON UPDATE "
          "CASCADE,[speed_max] INTEGER REFERENCES speed_limit(id) ON UPDATE "
          "CASCADE,PRIMARY KEY(way_id));";
      break;
    case TableNames::kTableWayNodes:
      sql =
          "CREATE TABLE [way_nodes] ([way_id] TEXT REFERENCES way(way_id) ON "
          "UPDATE CASCADE ON DELETE CASCADE,[node_index] "
          "TEXT,[data_line_number] TEXT,[node_value] TEXT);";
      break;
    case TableNames::kTableWayData:
      sql =
          "CREATE TABLE [way_data] ([way_id] TEXT REFERENCES way(way_id) ON "
          "UPDATE CASCADE ON DELETE CASCADE,[raw_data] BLOB,[navi_number] "
          "INTEGER,[navi_table_id] TEXT,PRIMARY KEY(way_id));";
      break;
    case TableNames::kTableNaviData:
      sql =
          "CREATE TABLE [navi_data] ([way_id] TEXT REFERENCES way(way_id) ON "
          "UPDATE CASCADE ON DELETE CASCADE,[navi_index] INTEGER,[data] "
          "BLOB);";
      break;
    default:
      break;
  }
  if (!sqlite_.ExcuteNonQuery(sql)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << "Create database table " << kTableNames.at(table_name)
           << " failed. " << err_msg;
    return false;
  }
  return true;
}

DBOperator::DBOperator() { OpenDatabase(); }

DBOperator::~DBOperator() { CloseDatabase(); }

bool DBOperator::OpenDatabase() {
  if (!sqlite_.Open(database_name)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    return false;
  }
  return true;
}

void DBOperator::CloseDatabase() { sqlite_.Close(); }

bool DBOperator::FillTableSpeedLimit() {
  std::vector<SpeedLimit> speed_limits;
  std::size_t speed_base = 30;
  std::size_t speed_step = 10;

  sqlite_.BeginTransaction();
  bool res = true;
  std::string sql = "INSERT INTO speed_limit(id,speed) VALUES(?,?)";
  SQLiteCommand cmd(&sqlite_, sql);

  for (std::size_t i = 1; i <= 13; i++) {
    cmd.BindParam(1, i);
    cmd.BindParam(2, speed_base + speed_step * (i - 1));
    if (!sqlite_.ExcuteNonQuery(&cmd)) {
      std::string err_msg;
      sqlite_.GetLastErrorMsg(&err_msg);
      AERROR << err_msg;
      res = false;
      break;
    }
  }
  cmd.Clear();
  if (res) {
    sqlite_.CommitTransaction();
    return true;
  } else {
    sqlite_.RollbackTransaction();
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << "Commit transaction failed. " << err_msg;
    return false;
  }
  return true;
}

bool DBOperator::InitDatabase() {
  if (IsTableExisting(TableNames::kTableWay)) {
    return true;
  }
  if (CreateTable(TableNames::kTableSpeedLimit) &&
      CreateTable(TableNames::kTableWay) &&
      CreateTable(TableNames::kTableWayNodes) &&
      CreateTable(TableNames::kTableWayData) &&
      CreateTable(TableNames::kTableNaviData) && FillTableSpeedLimit()) {
    return true;
  }
  return false;
}

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
