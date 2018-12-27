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
 * @brief This file provides the declaration of some classes which implemented
 * the database interface.
 */

#ifndef MODULES_TOOLS_NAVI_GENERATOR_BACKEND_DATABASE_DB_HELPER_H_
#define MODULES_TOOLS_NAVI_GENERATOR_BACKEND_DATABASE_DB_HELPER_H_

#include <cstddef>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "sqlite3.h"  //NOLINT

/**
 * @namespace apollo::navi_generator::database
 * @brief apollo::navi_generator::database
 */
namespace apollo {
namespace navi_generator {
namespace database {

enum SqliteDataType {
  kSqliteDataTypeInteger = SQLITE_INTEGER,
  kSqliteDataTypeFloat = SQLITE_FLOAT,
  kSqliteDataTypeText = SQLITE_TEXT,
  kSqliteDataTypeBlob = SQLITE_BLOB,
  kSqliteDataTypeNull = SQLITE_NULL,
};

typedef int (*QueryCallback)(void* param, int colume_count, char** column_value,
                             char** column_name);

class SQLite;

class SQLiteDataReader {
 public:
  explicit SQLiteDataReader(sqlite3_stmt* stmt = nullptr);
  ~SQLiteDataReader();

 public:
  /**
   * @brief Read a row of data.
   * @return  Return true for success.
   */
  bool Read();
  /**
   * @brief Close the reader(call after reading).
   */
  void Close();
  /**
   * @brief Get the number of columns.
   */
  int ColumnCount(void);
  /**
   * @brief Get the column name corresponding to the column index which starting
   * from 1.
   */
  void GetName(int col_index, std::string* const name);
  /**
   * @brief Get the data type of column corresponding to the column index which
   * starting from 1.
   */
  SqliteDataType GetDataType(int col_index);
  /**
   * @brief Get the value of column corresponding to the column index which
   * starting from 1.
   */
  void GetStringValue(int col_index, std::string* const value);
  /**
   * @brief Get the value of column corresponding to the column index which
   * starting from 1.
   */
  int GetIntValue(int col_index);
  /**
   * @brief Get the value of column corresponding to the column index which
   * starting from 1.
   */
  std::uint8_t GetUint8Value(int col_index);
  /**
   * @brief Get the value of column corresponding to the column index which
   * starting from 1.
   */
  std::int64_t GetInt64Value(int col_index);
  /**
   * @brief Get the value of column corresponding to the column index which
   * starting from 1. Stored uint64 as a blob in sqlite because it does not
   * support uint64 storage.
   */
  std::uint64_t GetUint64Value(int col_index);
  /**
   * @brief Get the value of column corresponding to the column index which
   * starting from 1.
   */
  double GetFloatValue(int col_index);
  /**
   * @brief Get the value of column corresponding to the column index which
   * starting from 1.
   */
  int GetBlobValue(int col_index, const unsigned char** blob_value);
  /**
   * @brief Get the value of column corresponding to the column index which
   * starting from 1.
   */
  int GetBlobValue(int col_index, std::vector<unsigned char>* const blob_value);

 private:
  sqlite3_stmt* stmt_;
};

class SQLiteCommand {
 public:
  explicit SQLiteCommand(SQLite* sqlite);
  SQLiteCommand(SQLite* sqlite, const std::string& sql);
  ~SQLiteCommand();

 public:
  /**
   * @brief Compiling a SQL statement.
   */
  bool SetCommandText(const std::string& sql);
  /**
   * @brief Bind the string value of the param to the param index which starting
   * from 1.
   */
  bool BindParam(int index, const std::string& value);
  /**
   * @brief Bind the int value of the param to the param index which starting
   * from 1.
   */
  bool BindParam(int index, const int value);
  /**
   * @brief Bind the int value of the param to the param index which starting
   * from 1.
   */
  bool BindParam(int index, const std::uint8_t value);
  /**
   * @brief Bind the double value of the param to the param index which starting
   * from 1.
   */
  bool BindParam(int index, const double value);
  /**
   * @brief Bind the uint64_t value of the param to the param index which
   * starting from 1. Stores uint64 as a blob in sqlite because it does not
   * support uint64 storage.
   */
  bool BindParam(int index, const std::uint64_t value);
  /**
   * @brief Bind the blob value of the param to the param index which starting
   * from 1.
   */
  bool BindParam(int index, const unsigned char* value, int len);
  /**
   * @brief Bind the blob value of the param to the param index which starting
   * from 1.
   */
  bool BindParam(int index, const std::vector<unsigned char>* const value);
  /**
   * @brief Bind the null value of the param to the param index which starting
   * from 1.
   */
  bool BindParam(int index);
  /**
   * @brief Evaluate a SQL statement.
   */
  bool Excute();
  /**
   * @brief Destroy a prepared statement object.
   */
  void Clear();

 private:
  SQLite* sqlite_;
  sqlite3_stmt* stmt_;
};

class SQLite {
 public:
  SQLite(void);
  ~SQLite(void);

 public:
  /**
   * @brief Opening a new database connection.
   */
  bool Open(const std::string& db_name);
  /**
   * @brief Closing a database connection.
   */
  void Close();

  /**
   * @brief Evaluate a SQL statement to implement update or deletion.
   */
  bool ExcuteNonQuery(const std::string& sql);
  bool ExcuteNonQuery(SQLiteCommand* cmd);

  /**
   * @brief Evaluate a SQL statement to implement query.
   */
  bool ExcuteQuery(const std::string& sql, SQLiteDataReader** const reader);

  bool ExcuteQuery(const std::string& sql, QueryCallback callback);

  /**
   * @brief Begin a transaction commition.
   */
  bool BeginTransaction();
  /**
   * @brief Commit a transaction.
   */
  bool CommitTransaction();
  /**
   * @brief Rollback a transaction.
   */
  bool RollbackTransaction();

  /**
   * @brief Get the last error message.
   */
  void GetLastErrorMsg(std::string* const err_msg);

 public:
  friend class SQLiteCommand;

 private:
  sqlite3* db_;
};

}  // namespace database
}  // namespace navi_generator
}  // namespace apollo

#endif  // MODULES_TOOLS_NAVI_GENERATOR_BACKEND_DATABASE_DB_HELPER_H_
