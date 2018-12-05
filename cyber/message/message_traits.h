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

#ifndef CYBER_MESSAGE_MESSAGE_TRAITS_H_
#define CYBER_MESSAGE_MESSAGE_TRAITS_H_

#include <string>

#include "cyber/base/macros.h"
#include "cyber/message/intra_message.h"
#include "cyber/message/protobuf_traits.h"
#include "cyber/message/py_message_traits.h"
#include "cyber/message/raw_message_traits.h"

namespace apollo {
namespace cyber {
namespace message {

DEFINE_TYPE_TRAIT(HasByteSize, ByteSize)
DEFINE_TYPE_TRAIT(HasType, TypeName)
DEFINE_TYPE_TRAIT(HasDescriptor, GetDescriptorString)
DEFINE_TYPE_TRAIT(HasSerializeToString, SerializeToString)
DEFINE_TYPE_TRAIT(HasParseFromString, ParseFromString)
DEFINE_TYPE_TRAIT(HasSerializeToArray, SerializeToArray)
DEFINE_TYPE_TRAIT(HasParseFromArray, ParseFromArray)

template <typename T>
class HasSerializer {
 public:
  static constexpr bool value =
      HasSerializeToString<T>::value && HasParseFromString<T>::value &&
      HasSerializeToArray<T>::value && HasParseFromArray<T>::value;
};

// avoid potential ODR violation
template <typename T>
constexpr bool HasSerializer<T>::value;

template <typename T>
typename std::enable_if<HasByteSize<T>::value, int>::type ByteSize(
    const T& message) {
  return message.ByteSize();
}

template <typename T>
typename std::enable_if<!HasByteSize<T>::value, int>::type ByteSize(
    const T& message) {
  (void)message;
  return -1;
}

template <typename T>
typename std::enable_if<HasParseFromArray<T>::value, bool>::type ParseFromArray(
    const void* data, int size, T* message) {
  return message->ParseFromArray(data, size);
}

template <typename T>
typename std::enable_if<!HasParseFromArray<T>::value, bool>::type
ParseFromArray(const void* data, int size, T* message) {
  return false;
}

template <typename T>
typename std::enable_if<HasParseFromString<T>::value, bool>::type
ParseFromString(const std::string& str, T* message) {
  return message->ParseFromString(str);
}

template <typename T>
typename std::enable_if<!HasParseFromString<T>::value, bool>::type
ParseFromString(const std::string& str, T* message) {
  return false;
}

template <typename T>
typename std::enable_if<HasSerializeToArray<T>::value, bool>::type
SerializeToArray(const T& message, void* data, int size) {
  return message.SerializeToArray(data, size);
}

template <typename T>
typename std::enable_if<!HasSerializeToArray<T>::value, bool>::type
SerializeToArray(const T& message, void* data, int size) {
  return false;
}

template <typename T>
typename std::enable_if<HasSerializeToString<T>::value, bool>::type
SerializeToString(const T& message, std::string* str) {
  return message.SerializeToString(str);
}

template <typename T>
typename std::enable_if<!HasSerializeToString<T>::value, bool>::type
SerializeToString(const T& message, std::string* str) {
  return false;
}

template <typename T,
          typename std::enable_if<HasType<T>::value, bool>::type = 0>
std::string MessageType(const T& message) {
  return T::TypeName();
}

template <typename T,
          typename std::enable_if<HasType<T>::value, bool>::type = 0>
std::string MessageType() {
  return T::TypeName();
}

template <typename T,
          typename std::enable_if<
              !HasType<T>::value &&
                  !std::is_base_of<google::protobuf::Message, T>::value,
              bool>::type = 0>
std::string MessageType() {
  return typeid(T).name();
}

template <typename T,
          typename std::enable_if<HasDescriptor<T>::value, bool>::type = 0>
void GetDescriptorString(const std::string& type, std::string* desc_str) {
  T::GetDescriptorString(type, desc_str);
}

template <typename T,
          typename std::enable_if<
              !HasDescriptor<T>::value &&
                  !std::is_base_of<google::protobuf::Message, T>::value,
              bool>::type = 0>
void GetDescriptorString(const std::string& type, std::string* desc_str) {}

template <typename MessageT,
          typename std::enable_if<
              !std::is_base_of<google::protobuf::Message, MessageT>::value,
              int>::type = 0>
void GetDescriptorString(const MessageT& message, std::string* desc_str) {}

}  // namespace message
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_MESSAGE_MESSAGE_TRAITS_H_
