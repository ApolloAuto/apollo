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

#ifndef CYBER_MESSAGE_PY_MESSAGE_TRAITS_H_
#define CYBER_MESSAGE_PY_MESSAGE_TRAITS_H_

#include <cassert>
#include <memory>
#include <string>

#include "cyber/message/protobuf_factory.h"
#include "cyber/message/py_message.h"

namespace apollo {
namespace cyber {
namespace message {

// Template specialization for RawMessage
inline bool SerializeToArray(const PyMessageWrap& message, void* data,
                             int size) {
  return message.SerializeToArray(data, size);
}

inline bool SerializeToString(const PyMessageWrap& message, std::string* str) {
  return message.SerializeToString(str);
}

inline bool ParseFromArray(const void* data, int size, PyMessageWrap* message) {
  return message->ParseFromArray(data, size);
}

inline bool ParseFromString(const std::string& str, PyMessageWrap* message) {
  return message->ParseFromString(str);
}

inline std::string MessageType(const PyMessageWrap& message) {
  (void)message;
  return PyMessageWrap::TypeName();
}

template <typename MessageT,
          typename std::enable_if<std::is_same<PyMessageWrap, MessageT>::value,
                                  int>::type = 0>
inline std::string MessageType() {
  return PyMessageWrap::TypeName();
}

template <typename MessageT,
          typename std::enable_if<std::is_same<PyMessageWrap, MessageT>::value,
                                  int>::type = 0>
inline void GetDescriptorString(const std::string& type,
                                std::string* desc_str) {
  ProtobufFactory::Instance()->GetDescriptorString(type, desc_str);
}

inline int ByteSize(const PyMessageWrap& message) { return message.ByteSize(); }

}  // namespace message
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_MESSAGE_PY_MESSAGE_TRAITS_H_
