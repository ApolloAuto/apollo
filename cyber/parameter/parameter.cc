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

#include "cyber/message/protobuf_factory.h"

namespace apollo {
namespace cyber {

using apollo::cyber::message::ProtobufFactory;

Parameter::Parameter() {
  param_.set_name("");
  param_.set_type(ParamType::NOT_SET);
}

Parameter::Parameter(const std::string& name) {
  param_.set_name(name);
  param_.set_type(proto::ParamType::NOT_SET);
}

Parameter::Parameter(const Parameter& parameter) {
  param_.CopyFrom(parameter.param_);
}

Parameter::Parameter(const std::string& name, const bool bool_value) {
  param_.set_name(name);
  param_.set_bool_value(bool_value);
  param_.set_type(ParamType::BOOL);
  param_.set_type_name("BOOL");
}

Parameter::Parameter(const std::string& name, const int int_value) {
  param_.set_name(name);
  param_.set_int_value(int_value);
  param_.set_type(ParamType::INT);
  param_.set_type_name("INT");
}

Parameter::Parameter(const std::string& name, const int64_t int_value) {
  param_.set_name(name);
  param_.set_int_value(int_value);
  param_.set_type(ParamType::INT);
  param_.set_type_name("INT");
}

Parameter::Parameter(const std::string& name, const float double_value) {
  param_.set_name(name);
  param_.set_double_value(double_value);
  param_.set_type(ParamType::DOUBLE);
  param_.set_type_name("DOUBLE");
}

Parameter::Parameter(const std::string& name, const double double_value) {
  param_.set_name(name);
  param_.set_double_value(double_value);
  param_.set_type(ParamType::DOUBLE);
  param_.set_type_name("DOUBLE");
}

Parameter::Parameter(const std::string& name, const std::string& string_value) {
  param_.set_name(name);
  param_.set_string_value(string_value);
  param_.set_type(ParamType::STRING);
  param_.set_type_name("STRING");
}

Parameter::Parameter(const std::string& name, const char* string_value) {
  param_.set_name(name);
  param_.set_string_value(string_value);
  param_.set_type(ParamType::STRING);
  param_.set_type_name("STRING");
}

Parameter::Parameter(const std::string& name, const std::string& msg_str,
                     const std::string& full_name,
                     const std::string& proto_desc) {
  param_.set_name(name);
  param_.set_string_value(msg_str);
  param_.set_type(ParamType::PROTOBUF);
  param_.set_type_name(full_name);
  param_.set_proto_desc(proto_desc);
}

Parameter::Parameter(const std::string& name,
                     const google::protobuf::Message& msg) {
  param_.set_name(name);
  std::string str;
  msg.SerializeToString(&str);
  std::string desc;
  ProtobufFactory::GetDescriptorString(msg, &desc);
  param_.set_string_value(str);
  param_.set_type(ParamType::PROTOBUF);
  param_.set_type_name(msg.GetDescriptor()->full_name());
  param_.set_proto_desc(desc);
}

void Parameter::FromProtoParam(const Param& param) { param_.CopyFrom(param); }

Param Parameter::ToProtoParam() const { return param_; }

std::string Parameter::DebugString() const {
  std::stringstream ss;
  ss << "{name: \"" << param_.name() << "\", ";
  ss << "type: \"" << TypeName() << "\", ";
  ss << "value: ";
  switch (Type()) {
    case ParamType::BOOL: {
      ss << (AsBool() ? "true" : "false");
      break;
    }
    case ParamType::INT: {
      ss << std::to_string(AsInt64());
      break;
    }
    case ParamType::DOUBLE: {
      ss << std::to_string(AsDouble());
      break;
    }
    case ParamType::STRING: {
      ss << "\"" << AsString() << "\"";
      break;
    }
    case ParamType::PROTOBUF: {
      ProtobufFactory::Instance()->RegisterMessage(Descriptor());
      auto message =
          ProtobufFactory::Instance()->GenerateMessageByType(TypeName());
      if (message != nullptr) {
        message->ParseFromString(AsString());
        ss << "\"" << message->ShortDebugString() << "\"";
        delete message;
      }
      break;
    }
    case ParamType::NOT_SET: {
      ss << "not set";
      break;
    }
    default:
      // do nothing
      break;
  }
  ss << "}";
  return ss.str();
}

}  // namespace cyber
}  // namespace apollo
