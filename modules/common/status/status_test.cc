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

#include "modules/common/status/status.h"

#include "gtest/gtest.h"

namespace apollo {
namespace common {

TEST(Status, OK) {
  EXPECT_EQ(Status::OK().code(), ErrorCode::OK);
  EXPECT_EQ(Status::OK().error_message(), "");
  EXPECT_EQ(Status::OK(), Status::OK());
  EXPECT_EQ(Status::OK(), Status());
  Status s;
  EXPECT_TRUE(s.ok());
}

TEST(Status, Set) {
  Status status;
  status = Status(ErrorCode::CONTROL_ERROR, "Error message");
  EXPECT_EQ(status.code(), ErrorCode::CONTROL_ERROR);
  EXPECT_EQ(status.error_message(), "Error message");
}

TEST(Status, Copy) {
  Status a(ErrorCode::CONTROL_ERROR, "Error message");
  Status b(a);
  EXPECT_EQ(a.ToString(), b.ToString());
}

TEST(Status, Assign) {
  Status a(ErrorCode::CONTROL_ERROR, "Error message");
  Status b;
  b = a;
  EXPECT_EQ(a.ToString(), b.ToString());
}

TEST(Status, EqualsSame) {
  Status a(ErrorCode::CONTROL_ERROR, "Error message");
  Status b(ErrorCode::CONTROL_ERROR, "Error message");
  EXPECT_EQ(a, b);
}

TEST(Status, EqualsDifferentCode) {
  const Status a(ErrorCode::CONTROL_ERROR, "Error message");
  const Status b(ErrorCode::CANBUS_ERROR, "Error message");
  EXPECT_NE(a, b);
}

TEST(Status, EqualsDifferentMessage) {
  const Status a(ErrorCode::CONTROL_ERROR, "Error message1");
  const Status b(ErrorCode::CONTROL_COMPUTE_ERROR, "Error message2");
  EXPECT_NE(a, b);
}

TEST(Status, ToString) {
  EXPECT_EQ("OK", Status().ToString());

  const Status a(ErrorCode::CONTROL_ERROR, "Error message");
  EXPECT_EQ("CONTROL_ERROR: Error message", a.ToString());
}

}  // namespace common
}  // namespace apollo
