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

#include "cyber/common/file.h"

#include <cstdlib>
#include <string>
#include <vector>
#include "gmock/gmock.h"
#include "gtest/gtest.h"

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

TEST(FileTest, ListSubPaths) {
  const auto root_subdirs = ListSubPaths("/");

  // Some common root subdirs should exist.
  EXPECT_NE(root_subdirs.end(),
            std::find(root_subdirs.begin(), root_subdirs.end(), "home"));
  EXPECT_NE(root_subdirs.end(),
            std::find(root_subdirs.begin(), root_subdirs.end(), "root"));
  // Something shouldn't exist.
  EXPECT_EQ(root_subdirs.end(),
            std::find(root_subdirs.begin(), root_subdirs.end(), "impossible"));
}

TEST(FileTest, Glob) {
  // Match none.
  EXPECT_TRUE(Glob("/path/impossible/*").empty());
  // Match one.
  EXPECT_THAT(Glob("/apollo"), testing::ElementsAre(std::string("/apollo")));
  EXPECT_THAT(Glob("/apol?o"), testing::ElementsAre(std::string("/apollo")));
  // Match multiple.
  EXPECT_THAT(
      Glob("/apol?o/modules/p*"),
      testing::AllOf(
          testing::Contains(std::string("/apollo/modules/perception")),
          testing::Contains(std::string("/apollo/modules/planning")),
          testing::Contains(std::string("/apollo/modules/prediction"))));
}

TEST(FileTest, GetAbsolutePath) {
  EXPECT_EQ("./xx.txt", GetAbsolutePath("", "./xx.txt"));
  EXPECT_EQ("/abc", GetAbsolutePath("/abc", ""));
  EXPECT_EQ("/home/work/xx.txt", GetAbsolutePath("/home/work", "xx.txt"));
  EXPECT_EQ("/home/work/xx.txt", GetAbsolutePath("/home/work/", "xx.txt"));
  EXPECT_EQ("/xx.txt", GetAbsolutePath("/home/work", "/xx.txt"));
  EXPECT_EQ("/home/work/./xx.txt", GetAbsolutePath("/home/work", "./xx.txt"));
}

TEST(FileTest, GetFileName) {
  EXPECT_EQ("xx.txt", GetFileName("xx.txt"));
  EXPECT_EQ("xx", GetFileName("./xx.txt", true));
  EXPECT_EQ("xx.txt", GetFileName("./xx.txt"));
  EXPECT_EQ("xx", GetFileName("./xx.txt", true));
  EXPECT_EQ(".txt", GetFileName("./.txt"));
  EXPECT_EQ("", GetFileName("./.txt", true));
  EXPECT_EQ("txt", GetFileName("/path/.to/txt"));
  EXPECT_EQ("txt", GetFileName("/path/.to/txt", true));
  EXPECT_EQ("", GetFileName("/path/to/"));
  EXPECT_EQ("", GetFileName("/path/to/", true));
}

}  // namespace common
}  // namespace cyber
}  // namespace apollo
