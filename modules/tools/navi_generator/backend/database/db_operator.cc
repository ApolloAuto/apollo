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

bool DBOperator::SaveWay(const Way& way) {
  bool res = true;
  std::string sql =
      "INSERT INTO way(way_id,pre_way_id,next_way_id,speed_min,speed_max) "
      "VALUES(?,?,?,?,?)";
  SQLiteCommand cmd(&sqlite_, sql);
  cmd.BindParam(1, way.way_id);
  (way.pre_way_id == 0) ? cmd.BindParam(2) : cmd.BindParam(2, way.pre_way_id);
  (way.next_way_id == 0) ? cmd.BindParam(3) : cmd.BindParam(3, way.next_way_id);
  cmd.BindParam(4, way.speed_min);
  cmd.BindParam(5, way.speed_max);

  if (!sqlite_.ExcuteNonQuery(&cmd)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    res = false;
  }
  cmd.Clear();
  return res;
}

bool DBOperator::SaveWayNodes(const WayNodes& way_nodes) {
  sqlite_.BeginTransaction();
  bool res = true;
  std::string sql =
      "INSERT INTO way_nodes(way_id,node_index,data_line_number,node_value) "
      "VALUES(?,?,?,?)";
  SQLiteCommand cmd(&sqlite_, sql);

  cmd.BindParam(1, way_nodes.way_id);
  for (auto& node : way_nodes.nodes) {
    cmd.BindParam(2, node.node_index);
    cmd.BindParam(3, node.data_line_number);
    cmd.BindParam(4, node.node_value);
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

bool DBOperator::SaveWayData(const WayData& way_data) {
  bool res = true;
  std::string sql =
      "INSERT INTO way_data(way_id,raw_data,navi_number,navi_table_id) "
      "VALUES(?,?,?,?)";
  SQLiteCommand cmd(&sqlite_, sql);
  cmd.BindParam(1, way_data.way_id);
  cmd.BindParam(2, way_data.raw_data.data(), way_data.raw_data.size());
  cmd.BindParam(3, way_data.navi_number);
  cmd.BindParam(4, way_data.navi_table_id);

  if (!sqlite_.ExcuteNonQuery(&cmd)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    res = false;
  }
  cmd.Clear();
  return res;
}

bool DBOperator::SaveNaviData(const NaviInfo& navi_info) {
  sqlite_.BeginTransaction();
  bool res = true;
  std::string sql =
      "INSERT INTO navi_data(way_id,navi_index,data) VALUES(?,?,?)";
  SQLiteCommand cmd(&sqlite_, sql);
  cmd.BindParam(1, navi_info.way_id);
  for (auto& data : navi_info.navi_data) {
    cmd.BindParam(2, data.navi_index);
    cmd.BindParam(3, data.data.data(), data.data.size());

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

bool DBOperator::QueryNaviDataWithWayId(
    const std::uint64_t way_id, std::vector<NaviData>* const navi_data) {
  CHECK_NOTNULL(navi_data);
  bool res = false;
  std::string sql = "SELECT navi_index, data FROM navi_data WHERE way_id = '" +
                    std::to_string(way_id) + "';";
  SQLiteDataReader* reader = nullptr;
  if (!sqlite_.ExcuteQuery(sql, &reader)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    return false;
  }
  while (reader->Read()) {
    NaviData data;
    data.navi_index = reader->GetIntValue(0);
    reader->GetBlobValue(1, &data.data);
    navi_data->emplace_back(data);
    res = true;
  }

  DeleteSQLiteDataReader(reader);
  return res;
}

bool DBOperator::QueryNaviDataWithWayId(const std::uint64_t way_id,
                                        const std::uint8_t navi_index,
                                        NaviData* const navi_data) {
  CHECK_NOTNULL(navi_data);
  bool res = false;
  std::string sql = "SELECT data FROM navi_data WHERE way_id = '" +
                    std::to_string(way_id) + "'AND navi_index='" +
                    std::to_string(navi_index) + "';";
  SQLiteDataReader* reader = nullptr;
  if (!sqlite_.ExcuteQuery(sql, &reader)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    return false;
  }
  while (reader->Read()) {
    navi_data->navi_index = reader->GetUint8Value(0);
    reader->GetBlobValue(1, &navi_data->data);
    res = true;
  }

  DeleteSQLiteDataReader(reader);
  return res;
}

bool DBOperator::QueryWayNodesWithWayId(const std::uint64_t way_id,
                                        WayNodes* const way_nodes) {
  CHECK_NOTNULL(way_nodes);
  bool res = false;
  std::string sql = "SELECT * FROM way_nodes WHERE way_id = '" +
                    std::to_string(way_id) + "';";
  SQLiteDataReader* reader = nullptr;
  if (!sqlite_.ExcuteQuery(sql, &reader)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    return false;
  }
  std::vector<Node> nodes;
  while (reader->Read()) {
    Node node;
    node.node_index = reader->GetUint64Value(1);
    node.data_line_number = reader->GetUint64Value(2);
    reader->GetStringValue(3, &node.node_value);
    nodes.emplace_back(node);
    res = true;
  }
  if (nodes.size() > 0) {
    way_nodes->way_id = way_id;
    way_nodes->nodes = nodes;
  }

  DeleteSQLiteDataReader(reader);
  return res;
}

bool DBOperator::QueryWayWithWayId(const std::uint64_t way_id, Way* const way) {
  CHECK_NOTNULL(way);
  bool res = false;
  std::string sql =
      "SELECT * FROM way WHERE way_id = '" + std::to_string(way_id) + "';";
  SQLiteDataReader* reader = nullptr;
  if (!sqlite_.ExcuteQuery(sql, &reader)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    return false;
  }
  apollo::navi_generator::database::SqliteDataType data_type;
  while (reader->Read()) {
    way->way_id = reader->GetUint64Value(0);
    data_type = reader->GetDataType(1);
    way->pre_way_id =
        (data_type ==
         apollo::navi_generator::database::SqliteDataType::kSqliteDataTypeNull)
            ? 0
            : reader->GetUint64Value(1);
    data_type = reader->GetDataType(2);
    way->next_way_id =
        (data_type ==
         apollo::navi_generator::database::SqliteDataType::kSqliteDataTypeNull)
            ? 0
            : reader->GetUint64Value(2);
    data_type = reader->GetDataType(3);
    way->speed_min =
        (data_type ==
         apollo::navi_generator::database::SqliteDataType::kSqliteDataTypeNull)
            ? 0
            : reader->GetUint64Value(3);
    data_type = reader->GetDataType(4);
    way->speed_max =
        (data_type ==
         apollo::navi_generator::database::SqliteDataType::kSqliteDataTypeNull)
            ? 0
            : reader->GetUint64Value(4);
    res = true;
  }

  DeleteSQLiteDataReader(reader);
  return res;
}

bool DBOperator::QueryWayDataWithWayId(const std::uint64_t way_id,
                                       WayData* const way_data) {
  CHECK_NOTNULL(way_data);
  bool res = false;
  std::string sql =
      "SELECT * FROM way_data WHERE way_id = '" + std::to_string(way_id) + "';";
  SQLiteDataReader* reader = nullptr;
  if (!sqlite_.ExcuteQuery(sql, &reader)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    return false;
  }
  while (reader->Read()) {
    way_data->way_id = reader->GetUint64Value(0);
    reader->GetBlobValue(1, &(way_data->raw_data));
    way_data->navi_number = reader->GetUint8Value(2);
    way_data->navi_table_id = reader->GetUint64Value(3);
    res = true;
  }

  DeleteSQLiteDataReader(reader);
  return res;
}

bool DBOperator::UpdateWay(const std::uint64_t way_id, const Way& way) {
  bool res = true;
  std::string sql =
      "UPDATE way SET pre_way_id=?,next_way_id=?,speed_min=?,speed_max=? WHERE "
      "way_id=?;";
  SQLiteCommand cmd(&sqlite_, sql);
  (way.pre_way_id == 0) ? cmd.BindParam(1) : cmd.BindParam(1, way.pre_way_id);
  (way.next_way_id == 0) ? cmd.BindParam(2) : cmd.BindParam(2, way.next_way_id);
  cmd.BindParam(3, way.speed_min);
  cmd.BindParam(4, way.speed_max);
  cmd.BindParam(5, way_id);

  if (!sqlite_.ExcuteNonQuery(&cmd)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    res = false;
  }
  cmd.Clear();
  return res;
}

bool DBOperator::UpdateWaySpeedLimit(const std::uint64_t way_id,
                                     const std::uint8_t speed_min,
                                     const std::uint8_t speed_max) {
  bool res = true;
  std::string sql =
      "UPDATE way SET speed_min = ?, speed_max = ? WHERE way_id = ?;";
  SQLiteCommand cmd(&sqlite_, sql);
  cmd.BindParam(1, speed_min);
  cmd.BindParam(2, speed_max);
  cmd.BindParam(3, way_id);

  if (!sqlite_.ExcuteNonQuery(&cmd)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    res = false;
  }
  cmd.Clear();
  return res;
}

bool DBOperator::UpdateWayNodes(const std::uint64_t way_id,
                                const WayNodes& way_nodes) {
  std::string sql = "DELETE FROM way_nodes WHERE way_id=?;";
  SQLiteCommand cmd(&sqlite_, sql);
  cmd.BindParam(1, way_id);

  if (!sqlite_.ExcuteNonQuery(&cmd)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    cmd.Clear();
    return false;
  }
  cmd.Clear();

  return SaveWayNodes(way_nodes);
}

bool DBOperator::UpdateWayData(const std::uint64_t way_id,
                               const WayData& way_data) {
  bool res = true;
  std::string sql =
      "UPDATE way_data SET raw_data=?,navi_number=?,navi_table_id=? WHERE "
      "way_id=?;";
  SQLiteCommand cmd(&sqlite_, sql);
  cmd.BindParam(1, &(way_data.raw_data));
  cmd.BindParam(2, way_data.navi_number);
  cmd.BindParam(3, way_data.navi_table_id);
  cmd.BindParam(4, way_id);

  if (!sqlite_.ExcuteNonQuery(&cmd)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    res = false;
  }
  cmd.Clear();
  return res;
}

bool DBOperator::UpdateNaviData(const std::uint64_t way_id,
                                const NaviInfo& navi_info) {
  std::string sql = "DELETE FROM navi_data WHERE way_id=?;";
  SQLiteCommand cmd(&sqlite_, sql);
  cmd.BindParam(1, way_id);

  if (!sqlite_.ExcuteNonQuery(&cmd)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    cmd.Clear();
    return false;
  }
  cmd.Clear();

  return SaveNaviData(navi_info);
}

bool DBOperator::DeleteWay(const std::uint64_t way_id) {
  std::string sql = "DELETE FROM way WHERE way_id=?;";
  SQLiteCommand cmd(&sqlite_, sql);
  cmd.BindParam(1, way_id);

  if (!sqlite_.ExcuteNonQuery(&cmd)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    cmd.Clear();
    return false;
  }
  cmd.Clear();
  if (!DeleteWayNodes(way_id)) {
    return false;
  }
  if (!DeleteWayData(way_id)) {
    return false;
  }
  if (!DeleteNaviData(way_id)) {
    return false;
  }
  return true;
}

bool DBOperator::DeleteWayNodes(const std::uint64_t way_id) {
  bool res = true;
  std::string sql = "DELETE FROM way_nodes WHERE way_id=?;";
  SQLiteCommand cmd(&sqlite_, sql);
  cmd.BindParam(1, way_id);

  if (!sqlite_.ExcuteNonQuery(&cmd)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    res = false;
  }

  cmd.Clear();
  return res;
}
bool DBOperator::DeleteWayData(const std::uint64_t way_id) {
  bool res = true;
  std::string sql = "DELETE FROM way_data WHERE way_id=?;";
  SQLiteCommand cmd(&sqlite_, sql);
  cmd.BindParam(1, way_id);

  if (!sqlite_.ExcuteNonQuery(&cmd)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    res = false;
  }

  cmd.Clear();
  return res;
}
bool DBOperator::DeleteNaviData(const std::uint64_t way_id) {
  bool res = true;
  std::string sql = "DELETE FROM navi_data WHERE way_id=?;";
  SQLiteCommand cmd(&sqlite_, sql);
  cmd.BindParam(1, way_id);

  if (!sqlite_.ExcuteNonQuery(&cmd)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    res = false;
  }

  cmd.Clear();
  return res;
}

bool DBOperator::CreateNewWayId(std::uint64_t* const way_id) {
  CHECK_NOTNULL(way_id);
  bool res = false;
  std::string sql = "SELECT max(way_id) from way;";
  SQLiteDataReader* reader = nullptr;
  if (!sqlite_.ExcuteQuery(sql, &reader)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    return false;
  }
  apollo::navi_generator::database::SqliteDataType data_type;
  while (reader->Read()) {
    data_type = reader->GetDataType(0);
    *way_id =
        (data_type ==
         apollo::navi_generator::database::SqliteDataType::kSqliteDataTypeNull)
            ? 1
            : reader->GetUint64Value(0) + 1;
    res = true;
  }
  DeleteSQLiteDataReader(reader);
  return res;
}

bool DBOperator::GetNaviTableId(std::uint64_t* const navi_table_id) {
  CHECK_NOTNULL(navi_table_id);
  bool res = false;
  std::uint64_t cur_max_table_id = 0;
  std::string sql = "SELECT max(navi_table_id) from way_data;";
  SQLiteDataReader* reader = nullptr;
  if (!sqlite_.ExcuteQuery(sql, &reader)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    return false;
  }
  apollo::navi_generator::database::SqliteDataType data_type;
  while (reader->Read()) {
    data_type = reader->GetDataType(0);
    cur_max_table_id =
        (data_type ==
         apollo::navi_generator::database::SqliteDataType::kSqliteDataTypeNull)
            ? 0
            : reader->GetUint64Value(0);
    res = true;
  }

  std::uint64_t table_line_counts = 0;
  sql = "SELECT count(*) from navi_data_" + std::to_string(cur_max_table_id) +
        ";";
  if (!sqlite_.ExcuteQuery(sql, &reader)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    DeleteSQLiteDataReader(reader);
    return false;
  }
  while (reader->Read()) {
    table_line_counts = reader->GetUint64Value(0);
    res = true;
  }

  DeleteSQLiteDataReader(reader);

  *navi_table_id = (table_line_counts < kMaxRowNumberOfDBTable)
                       ? cur_max_table_id
                       : cur_max_table_id + 1;

  return res;
}

bool DBOperator::QueryNaviWithPos(
    const std::string& node_value,
    std::vector<NaviInfoWithPos>* const navi_info) {
  CHECK_NOTNULL(navi_info);
  bool res = false;
  std::size_t find_times = 5;
  std::size_t cur_time = 0;
  std::string node_value_to_found = node_value;
  while (cur_time < find_times) {
    navi_info->clear();
    if (FindNaviWithPos(node_value_to_found, navi_info)) {
      res = true;
      break;
    } else {
      node_value_to_found =
          node_value_to_found.substr(0, node_value_to_found.size() - 1);
    }
  }

  return res;
}

bool DBOperator::FindNaviWithPos(
    const std::string& node_value,
    std::vector<NaviInfoWithPos>* const navi_info) {
  CHECK_NOTNULL(navi_info);
  bool res = false;
  std::string sql =
      "SELECT way_nodes.way_id, node_index, data_line_number,node_value, "
      "navi_index, data FROM way_nodes INNER JOIN navi_data ON "
      "way_nodes.way_id = navi_data.way_id WHERE node_value LIKE '" +
      node_value + "%' ORDER BY way_nodes.way_id,navi_index;";
  SQLiteDataReader* reader = nullptr;
  if (!sqlite_.ExcuteQuery(sql, &reader)) {
    std::string err_msg;
    sqlite_.GetLastErrorMsg(&err_msg);
    AERROR << err_msg;
    return false;
  }
  bool unique_way_id_has_been_read = false;
  std::uint64_t last_way_id = 0;
  NaviInfoWithPos navi_info_for_one_way;
  while (reader->Read()) {
    if (last_way_id == 0) {
      last_way_id = reader->GetUint64Value(0);
    }
    if (last_way_id != reader->GetUint64Value(0)) {
      // A new way
      navi_info->emplace_back(navi_info_for_one_way);
      navi_info_for_one_way.navi_data.clear();
      last_way_id = reader->GetUint64Value(0);
      unique_way_id_has_been_read = false;
    }
    if (!unique_way_id_has_been_read) {
      navi_info_for_one_way.way_id = reader->GetUint64Value(0);
      navi_info_for_one_way.node.node_index = reader->GetUint64Value(1);
      navi_info_for_one_way.node.data_line_number = reader->GetUint64Value(2);
      reader->GetStringValue(3, &(navi_info_for_one_way.node.node_value));
      unique_way_id_has_been_read = true;
    }
    NaviData navi_data;
    navi_data.navi_index = reader->GetUint8Value(4);
    reader->GetBlobValue(5, &(navi_data.data));
    navi_info_for_one_way.navi_data.emplace_back(navi_data);
    res = true;
  }
  // push back the last navi_info_for_one_way since it has not been pushed in
  // the while
  if (navi_info_for_one_way.navi_data.size() > 0) {
    navi_info->emplace_back(navi_info_for_one_way);
  }

  DeleteSQLiteDataReader(reader);
  return res;
}

bool DBOperator::QueryRouteWithStartEndWay(const std::uint64_t start_way_id,
                                           const std::uint64_t end_way_id,
                                           std::vector<Way>* const route) {
  CHECK_NOTNULL(route);
  if (FindRoute(start_way_id, end_way_id, Orientation::kOrientationForward,
                route)) {
    return true;
  }
  route->clear();
  return FindRoute(start_way_id, end_way_id, Orientation::kOrientationBackward,
                   route);
}

bool DBOperator::FindRoute(const std::uint64_t start_way_id,
                           const std::uint64_t end_way_id,
                           const Orientation orientation,
                           std::vector<Way>* const route) {
  CHECK_NOTNULL(route);
  bool ret = false;
  Way way;
  switch (orientation) {
    case Orientation::kOrientationForward: {
      if (!QueryWayWithWayId(start_way_id, &way)) {
        ret = false;
        break;
      }
      if (start_way_id == end_way_id) {
        route->emplace_back(way);
        ret = true;
        break;
      }
      if (way.next_way_id == 0) {
        ret = false;
        break;
      }
      if (way.next_way_id == end_way_id) {
        route->emplace_back(way);
        Way end_way;
        if (!QueryWayWithWayId(end_way_id, &end_way)) {
          ret = false;
          break;
        }
        route->emplace_back(end_way);
        ret = true;
        break;
      }
      route->emplace_back(way);
      ret = FindRoute(way.next_way_id, end_way_id, orientation, route);
    } break;
    case Orientation::kOrientationBackward: {
      if (!QueryWayWithWayId(start_way_id, &way)) {
        ret = false;
        break;
      }
      if (start_way_id == end_way_id) {
        route->emplace_back(way);
        ret = true;
        break;
      }
      if (way.pre_way_id == 0) {
        ret = false;
        break;
      }
      if (way.pre_way_id == end_way_id) {
        route->emplace_back(way);
        Way end_way;
        if (!QueryWayWithWayId(end_way_id, &end_way)) {
          ret = false;
          break;
        }
        route->emplace_back(end_way);
        ret = true;
        break;
      }
      route->emplace_back(way);
      ret = FindRoute(way.pre_way_id, end_way_id, orientation, route);
    } break;
    default:
      break;
  }

  return ret;
}

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
