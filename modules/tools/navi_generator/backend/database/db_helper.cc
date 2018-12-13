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
 * @brief This file provides the implementation of some classes which
 * implemented the database interface.
 */
#include "modules/tools/navi_generator/backend/database/db_helper.h"

#include "modules/common/log.h"

namespace apollo {
namespace navi_generator {
namespace database {

SQLite::SQLite(void) : db_(nullptr) {}

SQLite::~SQLite(void) { Close(); }

bool SQLite::Open(const std::string& db_name) {
  if (db_name.empty()) {
    return false;
  }
  if (sqlite3_open(db_name.data(), &db_) != SQLITE_OK) {
    return false;
  }
  return true;
}

void SQLite::Close() {
  if (db_) {
    sqlite3_close(db_);
  }
}

bool SQLite::ExcuteNonQuery(const std::string& sql) {
  if (sql.empty()) {
    return false;
  }
  sqlite3_stmt* stmt;
  if (sqlite3_prepare_v2(db_, sql.data(), -1, &stmt, nullptr) != SQLITE_OK) {
    return false;
  }
  sqlite3_step(stmt);

  return (sqlite3_finalize(stmt) == SQLITE_OK) ? true : false;
}

bool SQLite::ExcuteNonQuery(SQLiteCommand* cmd) {
  if (cmd == nullptr) {
    return false;
  }
  return cmd->Excute();
}

bool SQLite::ExcuteQuery(const std::string& sql, QueryCallback callback) {
  if (sql.empty() || callback == nullptr) {
    return false;
  }
  char* err_msg = nullptr;
  if (sqlite3_exec(db_, sql.data(), callback, nullptr, &err_msg) != SQLITE_OK) {
    AERROR << err_msg;
    return false;
  }
  return true;
}

bool SQLite::ExcuteQuery(const std::string& sql,
                         SQLiteDataReader** const reader) {
  CHECK_NOTNULL(reader);

  if (sql.empty()) {
    return false;
  }
  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(db_, sql.data(), -1, &stmt, nullptr) != SQLITE_OK) {
    return false;
  }

  *reader = new SQLiteDataReader(stmt);
  return true;
}

bool SQLite::BeginTransaction() {
  char* err_msg = nullptr;
  if (sqlite3_exec(db_, "BEGIN TRANSACTION;", nullptr, nullptr, &err_msg) !=
      SQLITE_OK) {
    AERROR << err_msg;
    return false;
  }
  return true;
}

bool SQLite::CommitTransaction() {
  char* err_msg = nullptr;
  if (sqlite3_exec(db_, "COMMIT TRANSACTION;;", nullptr, nullptr, &err_msg) !=
      SQLITE_OK) {
    AERROR << err_msg;
    return false;
  }
  return true;
}

bool SQLite::RollbackTransaction() {
  char* err_msg = nullptr;
  if (sqlite3_exec(db_, "ROLLBACK  TRANSACTION;", nullptr, nullptr, &err_msg) !=
      SQLITE_OK) {
    AERROR << err_msg;
    return false;
  }
  return true;
}

void SQLite::GetLastErrorMsg(std::string* const err_msg) {
  CHECK_NOTNULL(err_msg);
  const char* result = reinterpret_cast<const char*>(sqlite3_errmsg(db_));
  err_msg->assign(result);
  return;
}

SQLiteDataReader::SQLiteDataReader(sqlite3_stmt* stmt) : stmt_(stmt) {}

SQLiteDataReader::~SQLiteDataReader() { Close(); }

bool SQLiteDataReader::Read() {
  if (stmt_ == nullptr) {
    return false;
  }
  if (sqlite3_step(stmt_) != SQLITE_ROW) {
    return false;
  }
  return true;
}

void SQLiteDataReader::Close() {
  if (stmt_) {
    sqlite3_finalize(stmt_);
    stmt_ = nullptr;
  }
}

int SQLiteDataReader::ColumnCount(void) { return sqlite3_column_count(stmt_); }

void SQLiteDataReader::GetName(int col_index, std::string* const name) {
  CHECK_NOTNULL(name);
  const char* result =
      reinterpret_cast<const char*>(sqlite3_column_text(stmt_, col_index));
  name->assign(result);
  return;
}

SqliteDataType SQLiteDataReader::GetDataType(int col_index) {
  return static_cast<SqliteDataType>(sqlite3_column_type(stmt_, col_index));
}

void SQLiteDataReader::GetStringValue(int col_index, std::string* const value) {
  CHECK_NOTNULL(value);
  const char* result =
      reinterpret_cast<const char*>(sqlite3_column_text(stmt_, col_index));
  value->assign(result);
  return;
}

int SQLiteDataReader::GetIntValue(int col_index) {
  return sqlite3_column_int(stmt_, col_index);
}

std::uint8_t SQLiteDataReader::GetUint8Value(int col_index) {
  return static_cast<std::uint8_t>(sqlite3_column_int(stmt_, col_index));
}

std::int64_t SQLiteDataReader::GetInt64Value(int col_index) {
  return static_cast<std::int64_t>(sqlite3_column_int64(stmt_, col_index));
}

std::uint64_t SQLiteDataReader::GetUint64Value(int col_index) {
  std::string value;
  GetStringValue(col_index, &value);
  std::size_t idx = 0;
  return std::stoull(value, &idx, 0);
}

double SQLiteDataReader::GetFloatValue(int col_index) {
  return sqlite3_column_double(stmt_, col_index);
}

int SQLiteDataReader::GetBlobValue(int col_index,
                                   const unsigned char** blob_value) {
  *blob_value = (const unsigned char*)sqlite3_column_blob(stmt_, col_index);
  return sqlite3_column_bytes(stmt_, col_index);
}

int SQLiteDataReader::GetBlobValue(
    int col_index, std::vector<unsigned char>* const blob_value) {
  CHECK_NOTNULL(blob_value);
  const unsigned char* data = nullptr;
  int len = GetBlobValue(col_index, &data);
  blob_value->assign(data, data + len);
  return len;
}

SQLiteCommand::SQLiteCommand(SQLite* sqlite)
    : sqlite_(sqlite), stmt_(nullptr) {}

SQLiteCommand::SQLiteCommand(SQLite* sqlite, const std::string& sql)
    : sqlite_(sqlite), stmt_(nullptr) {
  SetCommandText(sql);
}

SQLiteCommand::~SQLiteCommand() {}

bool SQLiteCommand::SetCommandText(const std::string& sql) {
  if (sqlite3_prepare_v2(sqlite_->db_, sql.data(), -1, &stmt_, nullptr) !=
      SQLITE_OK) {
    return false;
  }
  return true;
}

bool SQLiteCommand::BindParam(int index, const std::string& value) {
  if (sqlite3_bind_text(stmt_, index, value.data(), -1, SQLITE_TRANSIENT) !=
      SQLITE_OK) {
    return false;
  }
  return true;
}

bool SQLiteCommand::BindParam(int index, const int value) {
  if (sqlite3_bind_int(stmt_, index, value) != SQLITE_OK) {
    return false;
  }
  return true;
}

bool SQLiteCommand::BindParam(int index, const std::uint8_t value) {
  return BindParam(index, static_cast<int>(value));
}

bool SQLiteCommand::BindParam(int index, const double value) {
  if (sqlite3_bind_double(stmt_, index, value) != SQLITE_OK) {
    return false;
  }
  return true;
}

bool SQLiteCommand::BindParam(int index, const std::uint64_t value) {
  std::string str = std::to_string(value);
  return BindParam(index, str);
}

bool SQLiteCommand::BindParam(int index, const unsigned char* value, int len) {
  CHECK_NOTNULL(value);
  if (sqlite3_bind_blob(stmt_, index, value, len, nullptr) != SQLITE_OK) {
    return false;
  }
  return true;
}

bool SQLiteCommand::BindParam(int index,
                              const std::vector<unsigned char>* const value) {
  CHECK_NOTNULL(value);
  return BindParam(index, value->data(), value->size());
}

bool SQLiteCommand::BindParam(int index) {
  if (sqlite3_bind_null(stmt_, index) != SQLITE_OK) {
    return false;
  }
  return true;
}

bool SQLiteCommand::Excute() {
  sqlite3_step(stmt_);
  return ((sqlite3_reset(stmt_) == SQLITE_OK) ? true : false);
}

void SQLiteCommand::Clear() {
  if (stmt_) {
    sqlite3_finalize(stmt_);
  }
}

}  // namespace database
}  // namespace navi_generator
}  // namespace apollo
