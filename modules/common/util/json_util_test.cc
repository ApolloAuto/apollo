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
#include "modules/common/proto/error_code.pb.h"

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

TEST(JsonUtilTest, GetStringFromJson) {
  Json json_obj;
  json_obj["key1"] = 0;
  json_obj["key2"] = "";
  json_obj["key3"] = "value2";

  std::string value;
  // No such key.
  EXPECT_FALSE(JsonUtil::GetStringFromJson(json_obj, "key0", &value));
  // Value is not string.
  EXPECT_FALSE(JsonUtil::GetStringFromJson(json_obj, "key1", &value));
  // Empty string.
  EXPECT_TRUE(JsonUtil::GetStringFromJson(json_obj, "key2", &value));
  EXPECT_TRUE(value.empty());
  // Non empty string.
  EXPECT_TRUE(JsonUtil::GetStringFromJson(json_obj, "key3", &value));
  EXPECT_EQ("value2", value);
}

TEST(JsonUtilTest, GetStringVectorFromJson) {
  Json json_obj;
  json_obj["key1"] = 0;
  json_obj["key2"] = Json::array();

  auto int_array = Json::array();
  int_array.push_back(0);
  json_obj["key3"] = int_array;

  auto mixed_array = Json::array();
  mixed_array.push_back(0);
  mixed_array.push_back("str1");
  json_obj["key4"] = mixed_array;

  auto str_array = Json::array();
  str_array.push_back("str1");
  str_array.push_back("str2");
  json_obj["key5"] = str_array;

  std::vector<std::string> value;
  // No such key.
  EXPECT_FALSE(JsonUtil::GetStringVectorFromJson(json_obj, "key0", &value));
  // Value is not array.
  EXPECT_FALSE(JsonUtil::GetStringVectorFromJson(json_obj, "key1", &value));
  // Empty array.
  EXPECT_TRUE(JsonUtil::GetStringVectorFromJson(json_obj, "key2", &value));
  EXPECT_TRUE(value.empty());
  // Non-string array.
  EXPECT_FALSE(JsonUtil::GetStringVectorFromJson(json_obj, "key3", &value));
  EXPECT_TRUE(value.empty());
  // Array contains non-string element.
  EXPECT_FALSE(JsonUtil::GetStringVectorFromJson(json_obj, "key4", &value));
  EXPECT_THAT(value, testing::ElementsAre("str1"));
  // String array.
  EXPECT_TRUE(JsonUtil::GetStringVectorFromJson(json_obj, "key5", &value));
  EXPECT_THAT(value, testing::ElementsAre("str1", "str2"));
}

}  // namespace util
}  // namespace common
}  // namespace apollo
