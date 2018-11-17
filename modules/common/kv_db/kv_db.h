/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_COMMON_KV_DB_KV_DB_H_
#define MODULES_COMMON_KV_DB_KV_DB_H_

#include <leveldb/db.h>
#include <memory>
#include <string>

/**
 * @namespace apollo::common
 * @brief apollo::common
 */
namespace apollo {
namespace common {

/**
 * @class KVDB
 *
 * @brief Lightweight key-value database to store system-wide parameters.
 *        We prefer keys like "apollo:data:commit_id".
 */
class KVDB {
 public:
  /**
   * @brief Store {key, value} to DB.
   * @param sync Whether flush right after writing.
   * @return Success or not.
   */
  static bool Put(const std::string &key, const std::string &value,
                  const bool sync = false);

  /**
   * @brief Delete a key.
   * @param sync Whether flush right after writing.
   * @return Success or not.
   */
  static bool Delete(const std::string &key,
                     const bool sync = false);

  static bool Has(const std::string &key);

  static std::string Get(const std::string &key,
                         const std::string &default_value = "");

 private:
  static std::unique_ptr<leveldb::DB> GetDB();
};

}  // namespace common
}  // namespace apollo

#endif  // MODULES_COMMON_KV_DB_KV_DB_H_
