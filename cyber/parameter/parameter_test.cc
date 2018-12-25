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

#include <gtest/gtest.h>
#include <string>

#include "cyber/cyber.h"
#include "cyber/message/message_traits.h"
#include "cyber/proto/parameter.pb.h"

namespace apollo {
namespace cyber {

using apollo::cyber::proto::Param;
using apollo::cyber::proto::ParamType;

class ParameterTest : public ::testing::Test {
 protected:
  ParameterTest() {}
  virtual ~ParameterTest() {}

  Parameter* _bool_param;
  Parameter* _int_param;
  Parameter* _double_param;
  Parameter* _string_param;
  Parameter* _protobuf_param;

  virtual void SetUp() {
    // Called before every TEST_F(ParameterTest, *)
    _bool_param = new Parameter("bool", true);
    _int_param = new Parameter("int", 100);
    _double_param = new Parameter("double", 0.0f);
    _string_param = new Parameter("string", "test");
    proto::Param param;
    param.set_name("param");
    std::string str;
    param.SerializeToString(&str);
    std::string desc;
    message::GetDescriptorString(param, &desc);
    std::string full_name = proto::Param::descriptor()->full_name();
    _protobuf_param = new Parameter("protobuf", str, full_name, desc);
  }

  virtual void TearDown() {
    // Called after every TEST_F(ParameterTest, *)
    delete _bool_param;
    delete _int_param;
    delete _double_param;
    delete _string_param;
  }
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
  proto::Param param = _int_param->ToProtoParam();
  EXPECT_EQ("int", param.name());
  EXPECT_EQ(ParamType::INT, param.type());
  EXPECT_EQ(100, param.int_value());
}

TEST_F(ParameterTest, type) {
  EXPECT_EQ(ParamType::BOOL, _bool_param->Type());
  EXPECT_EQ(ParamType::INT, _int_param->Type());
  EXPECT_EQ(ParamType::DOUBLE, _double_param->Type());
  EXPECT_EQ(ParamType::STRING, _string_param->Type());
  EXPECT_EQ(ParamType::PROTOBUF, _protobuf_param->Type());
}

TEST_F(ParameterTest, type_name) {
  EXPECT_EQ("BOOL", _bool_param->TypeName());
  EXPECT_EQ("INT", _int_param->TypeName());
  EXPECT_EQ("DOUBLE", _double_param->TypeName());
  EXPECT_EQ("STRING", _string_param->TypeName());
  EXPECT_EQ("apollo.cyber.proto.Param", _protobuf_param->TypeName());
}

TEST_F(ParameterTest, name) {
  EXPECT_EQ("bool", _bool_param->Name());
  EXPECT_EQ("int", _int_param->Name());
  EXPECT_EQ("double", _double_param->Name());
  EXPECT_EQ("string", _string_param->Name());
  EXPECT_EQ("protobuf", _protobuf_param->Name());
}

TEST_F(ParameterTest, as_bool) {
  EXPECT_TRUE(_bool_param->AsBool());
  EXPECT_FALSE(Parameter("bool", false).AsBool());
}

TEST_F(ParameterTest, as_int) { EXPECT_EQ(100, _int_param->AsInt64()); }

TEST_F(ParameterTest, as_double) { EXPECT_EQ(0.0, _double_param->AsDouble()); }

TEST_F(ParameterTest, AsString) {
  EXPECT_EQ(_string_param->AsString(), "test");
}

TEST_F(ParameterTest, value) {
  EXPECT_TRUE(_bool_param->value<bool>());
  EXPECT_EQ("", _bool_param->value<std::string>());

  EXPECT_EQ(100, _int_param->value<uint8_t>());
  EXPECT_EQ(100, _int_param->value<uint32_t>());
  EXPECT_EQ(100, _int_param->value<int64_t>());
  EXPECT_EQ(100, _int_param->value<int>());
  EXPECT_EQ("", _int_param->value<std::string>());

  EXPECT_EQ(0.0, _double_param->value<float>());
  EXPECT_EQ(0.0, _double_param->value<double>());
  EXPECT_EQ("", _double_param->value<std::string>());

  EXPECT_EQ("test", _string_param->value<std::string>());
  _string_param->value<int>();

  auto param = _protobuf_param->value<proto::Param>();
  EXPECT_EQ("protobuf", _protobuf_param->Name());
  EXPECT_EQ("apollo.cyber.proto.Param", _protobuf_param->TypeName());
  std::string str;
  param.SerializeToString(&str);
  EXPECT_EQ(str, _protobuf_param->value<std::string>());
  _protobuf_param->value<int>();
}

TEST_F(ParameterTest, debug_string) {
  EXPECT_EQ("{name: \"bool\", type: \"BOOL\", value: true}",
            _bool_param->DebugString());
  EXPECT_EQ("{name: \"int\", type: \"INT\", value: 100}",
            _int_param->DebugString());
  EXPECT_EQ("{name: \"double\", type: \"DOUBLE\", value: 0.000000}",
            _double_param->DebugString());
  EXPECT_EQ("{name: \"string\", type: \"STRING\", value: \"test\"}",
            _string_param->DebugString());
  EXPECT_EQ(
      "{name: \"protobuf\", type: \"apollo.cyber.proto.Param\", value: "
      "\"name: \"param\"\"}",
      _protobuf_param->DebugString());
}

}  // namespace cyber
}  // namespace apollo
