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

#include <string>

#include "gtest/gtest.h"

#include "cyber/proto/unit_test.pb.h"

namespace apollo {
namespace cyber {
namespace message {

TEST(ProtobufFactory, register_and_generate) {
  // register
  auto factory = ProtobufFactory::Instance();
  apollo::cyber::proto::ProtoDesc proto_desc;
  proto::UnitTest ut;
  EXPECT_FALSE(factory->RegisterMessage("test"));
  EXPECT_FALSE(factory->RegisterPythonMessage("test"));

  google::protobuf::FileDescriptorProto file_desc_proto;
  ut.GetDescriptor()->file()->CopyTo(&file_desc_proto);
  std::string file_desc_str;
  file_desc_proto.SerializeToString(&file_desc_str);
  EXPECT_TRUE(factory->RegisterPythonMessage(file_desc_str));

  proto_desc.set_desc(file_desc_str);
  std::string proto_desc_str;
  proto_desc.SerializeToString(&proto_desc_str);
  EXPECT_TRUE(factory->RegisterMessage(proto_desc_str));

  EXPECT_TRUE(factory->RegisterMessage(file_desc_proto));
  EXPECT_TRUE(factory->RegisterMessage(*(ut.GetDescriptor())));
  EXPECT_TRUE(factory->RegisterMessage(ut));

  // Get Descriptor
  std::string get_desc_str;
  ProtobufFactory::GetDescriptorString(ut, &get_desc_str);
  EXPECT_EQ(get_desc_str, proto_desc_str);

  get_desc_str.clear();
  ProtobufFactory::GetDescriptorString(ut.GetDescriptor(), &get_desc_str);
  EXPECT_EQ(get_desc_str, proto_desc_str);

  get_desc_str.clear();
  factory->GetDescriptorString("apollo.cyber.proto.UnitTest", &get_desc_str);
  EXPECT_EQ(get_desc_str, proto_desc_str);

  // Generate
  auto message = factory->GenerateMessageByType("test.not.found");
  EXPECT_EQ(nullptr, message);

  message = factory->GenerateMessageByType("apollo.cyber.proto.UnitTest");
  EXPECT_NE(nullptr, message);
  delete message;

  auto desc_ptr = factory->FindMessageTypeByName("test.not.found");
  EXPECT_EQ(nullptr, desc_ptr);

  desc_ptr = factory->FindMessageTypeByName("apollo.cyber.proto.UnitTest");
  EXPECT_NE(nullptr, desc_ptr);
}

}  // namespace message
}  // namespace cyber
}  // namespace apollo
