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

TEST_F(DBOperatorTest, InitDB) { EXPECT_TRUE(db_operator_->InitDatabase()); }

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
