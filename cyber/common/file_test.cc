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

#include <gtest/gtest.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "cyber/common/file.h"
#include "cyber/proto/unit_test.pb.h"

namespace apollo {
namespace cyber {
namespace common {

TEST(FileTest, proto_set_get_test) {
  apollo::cyber::proto::UnitTest message;
  message.set_class_name("FileTest");
  apollo::cyber::proto::UnitTest read_message;
  EXPECT_FALSE(SetProtoToASCIIFile(message, -1));
  EXPECT_FALSE(SetProtoToASCIIFile(message, "not_exists_dir/message.proto"));
  EXPECT_TRUE(SetProtoToASCIIFile(message, "message.ascii"));
  EXPECT_TRUE(SetProtoToBinaryFile(message, "message.binary"));

  EXPECT_FALSE(
      GetProtoFromASCIIFile("not_exists_dir/message.proto", &read_message));
  EXPECT_FALSE(
      GetProtoFromBinaryFile("not_exists_dir/message.proto", &read_message));
  EXPECT_TRUE(GetProtoFromASCIIFile("message.ascii", &read_message));
  EXPECT_TRUE(GetProtoFromBinaryFile("message.binary", &read_message));

  EXPECT_FALSE(GetProtoFromFile("not_exists_dir/message.proto", &read_message));
  EXPECT_TRUE(GetProtoFromFile("message.binary", &read_message));

  remove("message.binary");
  remove("message.ascii");
}

TEST(FileTest, file_utils_test) {
  apollo::cyber::proto::UnitTest message;
  message.set_class_name("FileTest");
  apollo::cyber::proto::UnitTest read_message;
  EXPECT_TRUE(SetProtoToBinaryFile(message, "message.binary"));

  std::string content;
  GetContent("message.binary", &content);
  EXPECT_FALSE(content.empty());
  content = "";
  GetContent("not_exists_dir/message.proto", &content);
  EXPECT_EQ("", content);

  EXPECT_EQ("/home/caros", GetAbsolutePath("/home/caros", ""));
  EXPECT_EQ("file.txt", GetAbsolutePath("", "file.txt"));
  EXPECT_EQ("/home/caros/README", GetAbsolutePath("", "/home/caros/README"));
  EXPECT_EQ("/home/caros/file.txt",
            GetAbsolutePath("/home/caros/", "file.txt"));
  EXPECT_EQ("/home/caros/file.txt", GetAbsolutePath("/home/caros", "file.txt"));
  EXPECT_EQ("/home/caros/../file.txt",
            GetAbsolutePath("/home/caros/", "../file.txt"));

  EXPECT_TRUE(PathExists("./"));
  EXPECT_FALSE(PathExists("not_exits_file"));

  EXPECT_TRUE(DirectoryExists("./"));
  EXPECT_FALSE(DirectoryExists("not_exits_file"));
  EXPECT_FALSE(DirectoryExists("message.binary"));

  EXPECT_FALSE(CopyFile("not_exists_file", "1.txt"));
  EXPECT_FALSE(CopyFile("message.binary", "/not_exists_file"));
  EXPECT_TRUE(CopyFile("message.binary", "message.binary.copy"));

  std::string current_path = GetCurrentPath();
  EXPECT_TRUE(EnsureDirectory(current_path));
  EXPECT_TRUE("/not_exists_dir");
  EXPECT_TRUE(EnsureDirectory(current_path + "/1"));
  EXPECT_TRUE(EnsureDirectory(current_path + "/1/2"));
  EXPECT_TRUE(EnsureDirectory(current_path + "/1/2/3"));
  EXPECT_TRUE(EnsureDirectory(current_path + "/2"));
  EXPECT_TRUE(CopyFile("message.binary", current_path + "/2/message.binary"));
  EXPECT_TRUE(DirectoryExists(current_path + "/1/2/3"));

  EXPECT_EQ("README", GetFileName("README"));
  EXPECT_EQ("README", GetFileName("/home/caros/README"));
  EXPECT_EQ("README", GetFileName("../../README"));

  std::vector<std::string> dirs = ListSubDirectories("/not_exists_directory");
  EXPECT_TRUE(dirs.empty());
  dirs = ListSubDirectories(current_path + "/1");

  EXPECT_FALSE(RemoveAllFiles("/not_exists_dir"));
  EXPECT_FALSE(RemoveAllFiles(current_path + "/1"));
  EXPECT_TRUE(RemoveAllFiles(current_path + "/2"));

  remove("message.binary");
  remove("message.binary.copy");
  rmdir((current_path + "/1/2/3").c_str());
  rmdir((current_path + "/1/2").c_str());
  rmdir((current_path + "/1").c_str());
  rmdir((current_path + "/2").c_str());
}

}  // namespace common
}  // namespace cyber
}  // namespace apollo
