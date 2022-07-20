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

#include "modules/common/util/json_util.h"

#include "gmock/gmock.h"
#include "google/protobuf/util/json_util.h"
#include "gtest/gtest.h"
#include "modules/common_msgs/basic_msgs/error_code.pb.h"

namespace apollo {
namespace common {
namespace util {

using Json = nlohmann::json;

TEST(JsonUtilTest, ProtoToTypedJson) {
  StatusPb status;
  status.set_msg("MsgA");

  Json json_obj = JsonUtil::ProtoToTypedJson("TypeA", status);
  EXPECT_EQ("TypeA", json_obj["type"]);
  EXPECT_EQ("MsgA", json_obj["data"]["msg"]);
}

TEST(JsonUtilTest, GetString) {
  Json json_obj = {{"int", 0}, {"empty_str", ""}, {"str", "value2"}};

  std::string value;
  // No such key.
  EXPECT_FALSE(JsonUtil::GetString(json_obj, "no_such_key", &value));
  // Value is not string.
  EXPECT_FALSE(JsonUtil::GetString(json_obj, "int", &value));
  // Empty string.
  EXPECT_TRUE(JsonUtil::GetString(json_obj, "empty_str", &value));
  EXPECT_TRUE(value.empty());
  // Non empty string.
  EXPECT_TRUE(JsonUtil::GetString(json_obj, "str", &value));
  EXPECT_EQ("value2", value);
}

TEST(JsonUtilTest, GetStringVector) {
  Json json_obj = {{"int", 0},
                   {"empty_array", Json::array()},
                   {"int_array", {0}},
                   {"any_array", {0, "str1"}},
                   {"str_array", {"str1", "str2"}}};

  std::vector<std::string> value;
  // No such key.
  EXPECT_FALSE(JsonUtil::GetStringVector(json_obj, "no_such_key", &value));
  // Value is not array.
  EXPECT_FALSE(JsonUtil::GetStringVector(json_obj, "int", &value));
  // Empty array.
  EXPECT_TRUE(JsonUtil::GetStringVector(json_obj, "empty_array", &value));
  EXPECT_TRUE(value.empty());
  // Non-string array.
  EXPECT_FALSE(JsonUtil::GetStringVector(json_obj, "int_array", &value));
  EXPECT_TRUE(value.empty());
  // Array contains non-string element.
  EXPECT_FALSE(JsonUtil::GetStringVector(json_obj, "any_array", &value));
  EXPECT_THAT(value, testing::ElementsAre("str1"));
  // String array.
  EXPECT_TRUE(JsonUtil::GetStringVector(json_obj, "str_array", &value));
  EXPECT_THAT(value, testing::ElementsAre("str1", "str2"));
}

}  // namespace util
}  // namespace common
}  // namespace apollo
