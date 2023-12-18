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

#include "cyber/message/protobuf_factory.h"

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace message {

using google::protobuf::MessageFactory;

ProtobufFactory::ProtobufFactory() {
  pool_.reset(new DescriptorPool());
  factory_.reset(new DynamicMessageFactory(pool_.get()));
}

ProtobufFactory::~ProtobufFactory() {
  factory_.reset();
  pool_.reset();
}

bool ProtobufFactory::RegisterMessage(
    const google::protobuf::Message& message) {
  const Descriptor* descriptor = message.GetDescriptor();
  return RegisterMessage(*descriptor);
}

bool ProtobufFactory::RegisterMessage(const Descriptor& desc) {
  FileDescriptorProto file_desc_proto;
  desc.file()->CopyTo(&file_desc_proto);
  return RegisterMessage(file_desc_proto);
}

bool ProtobufFactory::RegisterMessage(const ProtoDesc& proto_desc) {
  FileDescriptorProto file_desc_proto;
  file_desc_proto.ParseFromString(proto_desc.desc());

  // If the message in this proto file has been registered, return true.
  if (FindMessageTypeByFile(file_desc_proto)) {
    return true;
  }
  for (int i = 0; i < proto_desc.dependencies_size(); ++i) {
    auto dep = proto_desc.dependencies(i);
    if (!RegisterMessage(dep)) {
      return false;
    }
    FileDescriptorProto dep_file_desc_proto;
    dep_file_desc_proto.ParseFromString(dep.desc());
    const google::protobuf::Descriptor* descriptor =
        FindMessageTypeByFile(dep_file_desc_proto);

    // If descriptor is found, replace the dependency with registered path.
    if (descriptor != nullptr) {
      file_desc_proto.set_dependency(i, descriptor->file()->name());
    }
  }
  return RegisterMessage(file_desc_proto);
}

bool ProtobufFactory::RegisterPythonMessage(const std::string& proto_str) {
  FileDescriptorProto file_desc_proto;
  file_desc_proto.ParseFromString(proto_str);
  return RegisterMessage(file_desc_proto);
}

bool ProtobufFactory::RegisterMessage(const std::string& proto_desc_str) {
  ProtoDesc proto_desc;
  proto_desc.ParseFromString(proto_desc_str);
  return RegisterMessage(proto_desc);
}

// Internal method
bool ProtobufFactory::RegisterMessage(
    const FileDescriptorProto& file_desc_proto) {
  ErrorCollector ec;
  std::lock_guard<std::mutex> lg(register_mutex_);
  auto file_desc = pool_->BuildFileCollectingErrors(file_desc_proto, &ec);
  if (!file_desc) {
    /*
    AERROR << "Failed to register protobuf messages ["
              << file_desc_proto.name() << "]";
    */
    return false;
  }
  return true;
}

// Internal method
bool ProtobufFactory::GetProtoDesc(const FileDescriptor* file_desc,
                                   ProtoDesc* proto_desc) {
  FileDescriptorProto file_desc_proto;
  file_desc->CopyTo(&file_desc_proto);
  std::string str("");
  if (!file_desc_proto.SerializeToString(&str)) {
    return false;
  }

  proto_desc->set_desc(str);

  for (int i = 0; i < file_desc->dependency_count(); ++i) {
    auto desc = proto_desc->add_dependencies();
    if (!GetProtoDesc(file_desc->dependency(i), desc)) {
      return false;
    }
  }

  return true;
}

void ProtobufFactory::GetDescriptorString(const Descriptor* desc,
                                          std::string* desc_str) {
  ProtoDesc proto_desc;
  if (!GetProtoDesc(desc->file(), &proto_desc)) {
    AERROR << "Failed to get descriptor from message";
    return;
  }

  if (!proto_desc.SerializeToString(desc_str)) {
    AERROR << "Failed to get descriptor from message";
  }
}

void ProtobufFactory::GetDescriptorString(
    const google::protobuf::Message& message, std::string* desc_str) {
  const Descriptor* desc = message.GetDescriptor();
  return GetDescriptorString(desc, desc_str);
}

void ProtobufFactory::GetPythonDesc(const std::string& type,
                                    std::string* desc_str) {
  auto desc = pool_->FindMessageTypeByName(type);
  if (desc == nullptr) {
    return;
  }
  google::protobuf::DescriptorProto dp;
  desc->CopyTo(&dp);
  dp.SerializeToString(desc_str);
}

void ProtobufFactory::GetDescriptorString(const std::string& type,
                                          std::string* desc_str) {
  auto desc = DescriptorPool::generated_pool()->FindMessageTypeByName(type);
  if (desc != nullptr) {
    return GetDescriptorString(desc, desc_str);
  }

  desc = pool_->FindMessageTypeByName(type);
  if (desc == nullptr) {
    return;
  }
  return GetDescriptorString(desc, desc_str);
}

void ProtobufFactory::GetProtoPath(const std::string& type,
                                   std::string& location) {
  auto desc = DescriptorPool::generated_pool()->FindMessageTypeByName(type);
  if (desc != nullptr) {
    location = (desc->file())->name();
    return;
  }

  desc = pool_->FindMessageTypeByName(type);
  if (desc == nullptr) {
    return;
  }
  location = (desc->file())->name();
  return;
}

// Internal method
google::protobuf::Message* ProtobufFactory::GenerateMessageByType(
    const std::string& type) const {
  google::protobuf::Message* message = GetMessageByGeneratedType(type);
  if (message != nullptr) {
    return message;
  }

  const google::protobuf::Descriptor* descriptor =
      pool_->FindMessageTypeByName(type);
  if (descriptor == nullptr) {
    AERROR << "cannot find [" << type << "] descriptor";
    return nullptr;
  }

  const google::protobuf::Message* prototype =
      factory_->GetPrototype(descriptor);
  if (prototype == nullptr) {
    AERROR << "cannot find [" << type << "] prototype";
    return nullptr;
  }

  return prototype->New();
}

google::protobuf::Message* ProtobufFactory::GetMessageByGeneratedType(
    const std::string& type) const {
  auto descriptor =
      DescriptorPool::generated_pool()->FindMessageTypeByName(type);
  if (descriptor == nullptr) {
    // AERROR << "cannot find [" << type << "] descriptor";
    return nullptr;
  }

  auto prototype =
      MessageFactory::generated_factory()->GetPrototype(descriptor);
  if (prototype == nullptr) {
    // AERROR << "cannot find [" << type << "] prototype";
    return nullptr;
  }

  return prototype->New();
}

const Descriptor* ProtobufFactory::FindMessageTypeByName(
    const std::string& name) const {
  return pool_->FindMessageTypeByName(name);
}

const google::protobuf::ServiceDescriptor* ProtobufFactory::FindServiceByName(
    const std::string& name) const {
  return pool_->FindServiceByName(name);
}

const Descriptor* ProtobufFactory::FindMessageTypeByFile(
    const FileDescriptorProto& file_desc_proto) {
  const std::string& scope = file_desc_proto.package();
  std::string type;
  if (file_desc_proto.message_type_size()) {
    type = scope + "." + file_desc_proto.message_type(0).name();
  }
  const google::protobuf::Descriptor* descriptor =
      pool_->FindMessageTypeByName(type);
  return descriptor;
}

void ErrorCollector::AddError(const std::string& filename,
                              const std::string& element_name,
                              const google::protobuf::Message* descriptor,
                              ErrorLocation location,
                              const std::string& message) {
  UNUSED(element_name);
  UNUSED(descriptor);
  UNUSED(location);
  AWARN << "[" << filename << "] " << message;
}

void ErrorCollector::AddWarning(const std::string& filename,
                                const std::string& element_name,
                                const google::protobuf::Message* descriptor,
                                ErrorLocation location,
                                const std::string& message) {
  UNUSED(element_name);
  UNUSED(descriptor);
  UNUSED(location);
  AWARN << "[" << filename << "] " << message;
}

}  // namespace message
}  // namespace cyber
}  // namespace apollo
