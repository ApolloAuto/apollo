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

#include "cyber/parameter/parameter.h"

#include <string>
#include "gtest/gtest.h"

#include "cyber/cyber.h"
#include "cyber/message/message_traits.h"
#include "cyber/proto/parameter.pb.h"

namespace apollo {
namespace cyber {

using apollo::cyber::proto::Param;
using apollo::cyber::proto::ParamType;

class ParameterTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    // Called before every TEST_F(ParameterTest, *)
    bool_param_.reset(new Parameter("bool", true));
    int_param_.reset(new Parameter("int", 100));
    double_param_.reset(new Parameter("double", 0.0f));
    string_param_.reset(new Parameter("string", "test"));
    proto::Param param;
    param.set_name("param");
    std::string str;
    param.SerializeToString(&str);
    std::string desc;
    message::GetDescriptorString(param, &desc);
    std::string full_name = proto::Param::descriptor()->full_name();
    protobuf_param_.reset(new Parameter("protobuf", str, full_name, desc));
  }

  virtual void TearDown() {
    bool_param_.reset();
    int_param_.reset();
    double_param_.reset();
    string_param_.reset();
    protobuf_param_.reset();
  }

 protected:
  std::unique_ptr<Parameter> bool_param_;
  std::unique_ptr<Parameter> int_param_;
  std::unique_ptr<Parameter> double_param_;
  std::unique_ptr<Parameter> string_param_;
  std::unique_ptr<Parameter> protobuf_param_;
};

TEST_F(ParameterTest, constructors) {
  Parameter param("test");
  EXPECT_EQ(ParamType::NOT_SET, param.Type());
  Parameter param2(param);
  EXPECT_EQ(ParamType::NOT_SET, param2.Type());
}

TEST_F(ParameterTest, from_pb) {
  proto::Param param;
  param.set_name("test");
  param.set_type(proto::ParamType::INT);
  param.set_int_value(100);
  Parameter pv;
  pv.FromProtoParam(param);
  EXPECT_EQ("test", pv.Name());
  EXPECT_EQ(ParamType::INT, pv.Type());
  EXPECT_EQ(100, pv.AsInt64());

  param.set_name("test_bool");
  param.set_type(proto::ParamType::BOOL);
  param.set_bool_value(true);
  pv.FromProtoParam(param);
  EXPECT_TRUE(pv.AsBool());

  param.set_name("test_double");
  param.set_type(proto::ParamType::DOUBLE);
  param.set_double_value(10.00);
  pv.FromProtoParam(param);
  EXPECT_EQ(ParamType::DOUBLE, pv.Type());

  param.set_name("test_string");
  param.set_type(proto::ParamType::STRING);
  param.set_string_value("xxxxxx");
  pv.FromProtoParam(param);
  EXPECT_EQ("xxxxxx", pv.AsString());
}

TEST_F(ParameterTest, to_pb) {
  proto::Param param = int_param_->ToProtoParam();
  EXPECT_EQ("int", param.name());
  EXPECT_EQ(ParamType::INT, param.type());
  EXPECT_EQ(100, param.int_value());
}

TEST_F(ParameterTest, type) {
  EXPECT_EQ(ParamType::BOOL, bool_param_->Type());
  EXPECT_EQ(ParamType::INT, int_param_->Type());
  EXPECT_EQ(ParamType::DOUBLE, double_param_->Type());
  EXPECT_EQ(ParamType::STRING, string_param_->Type());
  EXPECT_EQ(ParamType::PROTOBUF, protobuf_param_->Type());
}

TEST_F(ParameterTest, type_name) {
  EXPECT_EQ("BOOL", bool_param_->TypeName());
  EXPECT_EQ("INT", int_param_->TypeName());
  EXPECT_EQ("DOUBLE", double_param_->TypeName());
  EXPECT_EQ("STRING", string_param_->TypeName());
  EXPECT_EQ("apollo.cyber.proto.Param", protobuf_param_->TypeName());
}

TEST_F(ParameterTest, name) {
  EXPECT_EQ("bool", bool_param_->Name());
  EXPECT_EQ("int", int_param_->Name());
  EXPECT_EQ("double", double_param_->Name());
  EXPECT_EQ("string", string_param_->Name());
  EXPECT_EQ("protobuf", protobuf_param_->Name());
}

TEST_F(ParameterTest, as_bool) {
  EXPECT_TRUE(bool_param_->AsBool());
  EXPECT_FALSE(Parameter("bool", false).AsBool());
}

TEST_F(ParameterTest, as_int) { EXPECT_EQ(100, int_param_->AsInt64()); }

TEST_F(ParameterTest, as_double) { EXPECT_EQ(0.0, double_param_->AsDouble()); }

TEST_F(ParameterTest, AsString) {
  EXPECT_EQ(string_param_->AsString(), "test");
}

TEST_F(ParameterTest, value) {
  EXPECT_TRUE(bool_param_->value<bool>());
  EXPECT_EQ("", bool_param_->value<std::string>());

  EXPECT_EQ(100, int_param_->value<uint8_t>());
  EXPECT_EQ(100, int_param_->value<uint32_t>());
  EXPECT_EQ(100, int_param_->value<int64_t>());
  EXPECT_EQ(100, int_param_->value<int>());
  EXPECT_EQ("", int_param_->value<std::string>());

  EXPECT_EQ(0.0, double_param_->value<float>());
  EXPECT_EQ(0.0, double_param_->value<double>());
  EXPECT_EQ("", double_param_->value<std::string>());

  EXPECT_EQ("test", string_param_->value<std::string>());
  string_param_->value<int>();

  auto param = protobuf_param_->value<proto::Param>();
  EXPECT_EQ("protobuf", protobuf_param_->Name());
  EXPECT_EQ("apollo.cyber.proto.Param", protobuf_param_->TypeName());
  std::string str;
  param.SerializeToString(&str);
  EXPECT_EQ(str, protobuf_param_->value<std::string>());
  protobuf_param_->value<int>();
}

TEST_F(ParameterTest, debug_string) {
  EXPECT_EQ("{name: \"bool\", type: \"BOOL\", value: true}",
            bool_param_->DebugString());
  EXPECT_EQ("{name: \"int\", type: \"INT\", value: 100}",
            int_param_->DebugString());
  EXPECT_EQ("{name: \"double\", type: \"DOUBLE\", value: 0.000000}",
            double_param_->DebugString());
  EXPECT_EQ("{name: \"string\", type: \"STRING\", value: \"test\"}",
            string_param_->DebugString());
  EXPECT_EQ(
      "{name: \"protobuf\", type: \"apollo.cyber.proto.Param\", value: "
      "\"name: \"param\"\"}",
      protobuf_param_->DebugString());
}

}  // namespace cyber
}  // namespace apollo
