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

#ifndef CYBER_PARAMETER_PARAMETER_H_
#define CYBER_PARAMETER_PARAMETER_H_

#include <string>

#include "cyber/common/log.h"
#include "cyber/proto/parameter.pb.h"

/**
 * @namespace cyber.parameter
 */
namespace apollo {
namespace cyber {

using apollo::cyber::proto::Param;
using apollo::cyber::proto::ParamType;

/**
 * @class Parameter
 * @brief A `Parameter` holds an apollo::cyber::proto::Param,
 * It's more human-readable, you can use basic-value type and Protobuf values
 * to construct a paramter. Parameter is identified by their `name`,
 * and you can get Parameter content by call value()
 */
class Parameter {
 public:
  /**
   * @brief Empty contructor
   */
  Parameter();
  /**
   * @brief copy-constructor
   */
  explicit Parameter(const Parameter& parameter);
  /**
   * @brief Just point the paramter's name
   */
  explicit Parameter(const std::string& name);
  /**
   * @brief use a bool type value to construct the parameter
   */
  Parameter(const std::string& name, const bool bool_value);
  /**
   * @brief use a int type value to construct the parameter
   */
  Parameter(const std::string& name, const int int_value);
  /**
   * @brief use a int64_t type value to construct the parameter
   */
  Parameter(const std::string& name, const int64_t int_value);
  /**
   * @brief use a float type value to construct the parameter
   */
  Parameter(const std::string& name, const float doule_value);
  /**
   * @brief use a double type value to construct the parameter
   */
  Parameter(const std::string& name, const double double_value);
  /**
   * @brief use a string type value to construct the parameter
   */
  Parameter(const std::string& name, const std::string& string_value);
  /**
   * @brief use a char* type value to construct the parameter
   */
  Parameter(const std::string& name, const char* string_value);
  /**
   * @brief use a protobuf type value to construct the parameter
   */
  Parameter(const std::string& name, const std::string& msg_str,
            const std::string& full_name, const std::string& proto_desc);
  /**
   * @brief use a google::protobuf::Message type value to construct the
   * parameter
   */
  Parameter(const std::string& name, const google::protobuf::Message& msg);

  /**
   * @brief Parse a cyber::proto::Param object to
   * cyber::parameter::Parameter object
   * @param param The cyber::proto::Param object parse from
   * @param parameter A pointer to the target Parameter object
   * @return True if parse ok, otherwise False
   */
  void FromProtoParam(const Param& param);

  /**
   * @brief Parse a cyber::parameter::Parameter object to
   * cyber::proto::Param object
   * @return The target cyber::proto::Param object
   */
  Param ToProtoParam() const;

  /**
   * @brief Get the cyber:parameter::ParamType of this object
   * @return result cyber:parameter::ParameterType
   */
  inline ParamType Type() const;

  ///< Get Paramter's type name, i.e. INT,DOUBLE,STRING or protobuf message's
  ///< fullname
  inline std::string TypeName() const;

  ///< Get Paramter's descriptor, only work on protobuf types
  inline std::string Descriptor() const;

  ///< Get Parameter's name
  inline const std::string Name() const;

  ///< Get Paramter as a bool value
  inline bool AsBool() const;
  ///< Get Paramter as a int64_t value
  inline int64_t AsInt64() const;
  ///< Get Paramter as a double value
  inline double AsDouble() const;
  ///< Get Paramter as a string value
  inline const std::string AsString() const;
  ///< show debug string
  std::string DebugString() const;

  ///< Translate paramter value as a protobuf::Message
  template <typename ValueType>
  typename std::enable_if<
      std::is_base_of<google::protobuf::Message, ValueType>::value,
      ValueType>::type
  value() const;

  ///< Translate paramter value to int type
  template <typename ValueType>
  typename std::enable_if<std::is_integral<ValueType>::value &&
                              !std::is_same<ValueType, bool>::value,
                          ValueType>::type
  value() const;

  ///< Translate paramter value to bool type
  template <typename ValueType>
  typename std::enable_if<std::is_floating_point<ValueType>::value,
                          ValueType>::type
  value() const;

  ///< Translate paramter value to string type
  template <typename ValueType>
  typename std::enable_if<std::is_convertible<ValueType, std::string>::value,
                          const std::string&>::type
  value() const;

  ///< Translate paramter value to bool type
  template <typename ValueType>
  typename std::enable_if<std::is_same<ValueType, bool>::value, bool>::type
  value() const;

 private:
  Param param_;
};

template <typename ValueType>
typename std::enable_if<
    std::is_base_of<google::protobuf::Message, ValueType>::value,
    ValueType>::type
Parameter::value() const {
  ValueType message;
  if (!message.ParseFromString(param_.string_value())) {
    AERROR << "The type of parameter \"" << param_.name() << "\" is "
           << TypeName() << ", not " << ValueType::descriptor()->full_name();
  }
  return message;
}

template <typename ValueType>
typename std::enable_if<std::is_integral<ValueType>::value &&
                            !std::is_same<ValueType, bool>::value,
                        ValueType>::type
Parameter::value() const {
  if (param_.type() != proto::ParamType::INT) {
    AERROR << "The type of parameter \"" << param_.name() << "\" is "
           << TypeName() << ", not INT";
  }
  return static_cast<ValueType>(param_.int_value());
}

template <typename ValueType>
typename std::enable_if<std::is_floating_point<ValueType>::value,
                        ValueType>::type
Parameter::value() const {
  if (param_.type() != proto::ParamType::DOUBLE) {
    AERROR << "The type of parameter \"" << param_.name() << "\" is "
           << TypeName() << ", not DOUBLE";
  }
  return static_cast<ValueType>(param_.double_value());
}

template <typename ValueType>
typename std::enable_if<std::is_convertible<ValueType, std::string>::value,
                        const std::string&>::type
Parameter::value() const {
  if (param_.type() != proto::ParamType::STRING &&
      param_.type() != proto::ParamType::PROTOBUF) {
    AERROR << "The type of parameter \"" << param_.name() << "\" is "
           << TypeName() << ", not STRING";
  }
  return param_.string_value();
}

template <typename ValueType>
typename std::enable_if<std::is_same<ValueType, bool>::value, bool>::type
Parameter::value() const {
  if (param_.type() != proto::ParamType::BOOL) {
    AERROR << "The type of parameter \"" << param_.name() << "\" is "
           << TypeName() << ", not BOOL";
  }
  return param_.bool_value();
}

inline ParamType Parameter::Type() const { return param_.type(); }

inline std::string Parameter::TypeName() const { return param_.type_name(); }

inline std::string Parameter::Descriptor() const { return param_.proto_desc(); }

inline const std::string Parameter::Name() const { return param_.name(); }

///<
inline bool Parameter::AsBool() const { return value<bool>(); }

inline int64_t Parameter::AsInt64() const { return value<int64_t>(); }

inline double Parameter::AsDouble() const { return value<double>(); }

const std::string Parameter::AsString() const { return value<std::string>(); }

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_PARAMETER_PARAMETER_H_
