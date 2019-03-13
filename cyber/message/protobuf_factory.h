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

#ifndef CYBER_MESSAGE_PROTOBUF_FACTORY_H_
#define CYBER_MESSAGE_PROTOBUF_FACTORY_H_

#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include "cyber/common/macros.h"
#include "cyber/proto/proto_desc.pb.h"
#include "google/protobuf/compiler/parser.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/dynamic_message.h"
#include "google/protobuf/io/tokenizer.h"

namespace apollo {
namespace cyber {
namespace message {

using apollo::cyber::proto::ProtoDesc;
using google::protobuf::Descriptor;
using google::protobuf::DescriptorPool;
using google::protobuf::DynamicMessageFactory;
using google::protobuf::FileDescriptor;
using google::protobuf::FileDescriptorProto;

class ErrorCollector : public google::protobuf::DescriptorPool::ErrorCollector {
  using ErrorLocation =
      google::protobuf::DescriptorPool::ErrorCollector::ErrorLocation;
  void AddError(const std::string& filename, const std::string& element_name,
                const google::protobuf::Message* descriptor,
                ErrorLocation location, const std::string& message) override;

  void AddWarning(const std::string& filename, const std::string& element_name,
                  const google::protobuf::Message* descriptor,
                  ErrorLocation location, const std::string& message) override;
};

class ProtobufFactory {
 public:
  ~ProtobufFactory();

  // Recursively register FileDescriptorProto and all its dependencies to
  // factory.
  bool RegisterMessage(const std::string& proto_desc_str);
  bool RegisterPythonMessage(const std::string& proto_str);

  // Convert the serialized FileDescriptorProto to real descriptors and place
  // them in factory.
  // It is an error if a FileDescriptorProto contains references to types or
  // other files that are not found in the Factory.
  bool RegisterMessage(const google::protobuf::Message& message);
  bool RegisterMessage(const Descriptor& desc);
  bool RegisterMessage(const FileDescriptorProto& file_desc_proto);

  // Serialize all descriptors of the given message to string.
  static void GetDescriptorString(const google::protobuf::Message& message,
                                  std::string* desc_str);

  // Serialize all descriptors of the descriptor to string.
  static void GetDescriptorString(const Descriptor* desc,
                                  std::string* desc_str);

  // Get Serialized descriptors of messages with the given type.
  void GetDescriptorString(const std::string& type, std::string* desc_str);

  // Given a type name, constructs the default (prototype) Message of that type.
  // Returns nullptr if no such message exists.
  google::protobuf::Message* GenerateMessageByType(
      const std::string& type) const;

  // Find a top-level message type by name. Returns nullptr if not found.
  const Descriptor* FindMessageTypeByName(const std::string& type) const;

  // Find a service definition by name. Returns nullptr if not found.
  const google::protobuf::ServiceDescriptor* FindServiceByName(
      const std::string& name) const;

  void GetPythonDesc(const std::string& type, std::string* desc_str);

 private:
  bool RegisterMessage(const ProtoDesc& proto_desc);
  google::protobuf::Message* GetMessageByGeneratedType(
      const std::string& type) const;
  static bool GetProtoDesc(const FileDescriptor* file_desc,
                           ProtoDesc* proto_desc);

  std::mutex register_mutex_;
  std::unique_ptr<DescriptorPool> pool_ = nullptr;
  std::unique_ptr<DynamicMessageFactory> factory_ = nullptr;

  DECLARE_SINGLETON(ProtobufFactory);
};

}  // namespace message
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_MESSAGE_PROTOBUF_FACTORY_H_
