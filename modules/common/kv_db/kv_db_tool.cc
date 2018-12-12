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
#include "modules/common/kv_db/kv_db.h"

#include <iostream>

#include "gflags/gflags.h"

#include "cyber/common/log.h"

DEFINE_string(op, "get", "Operation to execute, should be put, get or del.");
DEFINE_string(key, "", "The key to query.");
DEFINE_string(value, "", "The value to query.");

using apollo::common::KVDB;

int main(int32_t argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_key.empty()) {
    AFATAL << "Please specify --key.";
  }

  if (FLAGS_op == "get") {
    std::cout << KVDB::Get(FLAGS_key);
  } else if (FLAGS_op == "put") {
    if (FLAGS_value.empty()) {
      AFATAL << "Please specify --value.";
    }
    std::cout << KVDB::Put(FLAGS_key, FLAGS_value);
  } else if (FLAGS_op == "del") {
    std::cout << KVDB::Delete(FLAGS_key);
  } else {
    AFATAL << "Unknown op: " << FLAGS_op;
  }

  return 0;
}
