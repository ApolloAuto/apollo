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
#include "modules/common/kv_db/kv_db.h"

#include <leveldb/env.h>
#include <leveldb/options.h>

#include "gflags/gflags.h"
#include "modules/common/util/file.h"
#include "modules/common/util/util.h"

DEFINE_string(kv_db_path, "/apollo/data/kv_db", "Path to param DB file.");

namespace apollo {
namespace common {
namespace {

class BlockingEnv : public leveldb::EnvWrapper {
 public:
  BlockingEnv() : leveldb::EnvWrapper(leveldb::Env::Default()) {}

  // Block on trying to lock file.
  leveldb::Status LockFile(const std::string& fname, leveldb::FileLock** lock) {
    constexpr unsigned int USLEEP_INTERVAL = 100 * 1000;  // 0.1 second.
    while (!leveldb::EnvWrapper::LockFile(fname, lock).ok()) {
      AINFO_EVERY(100) << "Trying to get KVDB lock.";
      usleep(USLEEP_INTERVAL);
    }
    return leveldb::Status::OK();
  }
};

leveldb::Options DBOptions() {
  leveldb::Options options;
  options.create_if_missing = true;
  options.env = new BlockingEnv();
  return options;
}

}  // namespace

std::unique_ptr<leveldb::DB> KVDB::GetDB() {
  static const auto options = DBOptions();
  leveldb::DB *db = nullptr;
  CHECK(apollo::common::util::EnsureDirectory(FLAGS_kv_db_path));
  const auto status = leveldb::DB::Open(options, FLAGS_kv_db_path, &db);
  CHECK(status.ok()) << "Unable to open DB path " << FLAGS_kv_db_path
                     << "\n" << status.ToString();
  return std::unique_ptr<leveldb::DB>(db);
}

bool KVDB::Put(const std::string &key, const std::string &value,
               const bool sync) {
  leveldb::WriteOptions options;
  options.sync = sync;

  const auto status = GetDB()->Put(options, key, value);
  AERROR_IF(!status.ok()) << status.ToString();
  return status.ok();
}

bool KVDB::Delete(const std::string &key, const bool sync) {
  leveldb::WriteOptions options;
  options.sync = sync;

  const auto status = GetDB()->Delete(options, key);
  AERROR_IF(!status.ok()) << status.ToString();
  return status.ok();
}

bool KVDB::Has(const std::string &key) {
  static leveldb::ReadOptions options;

  std::string value;
  const auto status = GetDB()->Get(options, key, &value);
  CHECK(status.ok() || status.IsNotFound()) << status.ToString();
  return status.ok();
}

std::string KVDB::Get(const std::string &key,
                      const std::string &default_value) {
  static leveldb::ReadOptions options;

  std::string value;
  const auto status = GetDB()->Get(options, key, &value);
  CHECK(status.ok() || status.IsNotFound()) << status.ToString();
  return status.ok() ? value : default_value;
}

}  // namespace common
}  // namespace apollo
