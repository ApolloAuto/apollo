/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/common/util/file.h"

#include "gtest/gtest.h"
#include "modules/common/log.h"
#include "modules/common/util/testdata/simple.pb.h"

namespace apollo {
namespace common {
namespace util {

class FileTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    system("exec rm -rf ${TEST_TMPDIR}/*");
    temp_dir = std::getenv("TEST_TMPDIR");
  }

  std::string FilePath(const std::string &file_name) {
    return temp_dir + "/" + file_name;
  }

  std::string temp_dir;
};

TEST_F(FileTest, GetSetASCIIFile) {
  const std::string path = FilePath("output.pb.txt");

  test::SimpleMessage message;
  message.set_integer(17);
  message.set_text("This is some piece of text.");

  EXPECT_TRUE(SetProtoToASCIIFile(message, path));

  test::SimpleMessage read_message;
  EXPECT_TRUE(GetProtoFromASCIIFile(path, &read_message));

  EXPECT_EQ(message.integer(), read_message.integer());
  EXPECT_EQ(message.text(), read_message.text());
}

TEST_F(FileTest, GetSetBinaryFile) {
  const std::string path = FilePath("output.pb.bin");

  test::SimpleMessage message;
  message.set_integer(17);
  message.set_text("This is some piece of text.");

  EXPECT_TRUE(SetProtoToBinaryFile(message, path));

  test::SimpleMessage read_message;
  EXPECT_TRUE(GetProtoFromBinaryFile(path, &read_message));

  EXPECT_EQ(message.integer(), read_message.integer());
  EXPECT_EQ(message.text(), read_message.text());
}

TEST_F(FileTest, PathExists) {
  EXPECT_TRUE(PathExists("/root"));
  EXPECT_FALSE(PathExists("/something_impossible"));
}

TEST_F(FileTest, EnsureAndRemoveDirectory) {
  const std::string directory_path = FilePath("my_directory/haha/hehe");
  EXPECT_FALSE(DirectoryExists(directory_path));
  EXPECT_TRUE(EnsureDirectory(directory_path));
  EXPECT_TRUE(DirectoryExists(directory_path));
}

TEST_F(FileTest, RemoveAllFiles) {
  test::SimpleMessage message;
  message.set_integer(17);
  message.set_text("This is some piece of text.");

  const std::string path1 = FilePath("1.pb.txt");
  EXPECT_TRUE(SetProtoToASCIIFile(message, path1));

  const std::string path2 = FilePath("2.pb.txt");
  EXPECT_TRUE(SetProtoToASCIIFile(message, path2));

  EXPECT_TRUE(RemoveAllFiles(FilePath("")));
  EXPECT_FALSE(GetProtoFromASCIIFile(path1, &message));
  EXPECT_FALSE(GetProtoFromASCIIFile(path2, &message));
}

TEST_F(FileTest, ListSubDirectories) {
  // Expect {'modules/common/util/testdata'}
  const auto root_subdirs = ListSubDirectories("/");

  // Some common root subdirs should exist.
  EXPECT_NE(root_subdirs.end(),
            std::find(root_subdirs.begin(), root_subdirs.end(), "home"));
  EXPECT_NE(root_subdirs.end(),
            std::find(root_subdirs.begin(), root_subdirs.end(), "root"));
  // Something shouldn't exist.
  EXPECT_EQ(root_subdirs.end(),
            std::find(root_subdirs.begin(), root_subdirs.end(), "impossible"));
}

TEST_F(FileTest, GetAbsolutePath) {
  EXPECT_EQ("./xx.txt", GetAbsolutePath("", "./xx.txt"));
  EXPECT_EQ("/abc", GetAbsolutePath("/abc", ""));
  EXPECT_EQ("/home/work/xx.txt", GetAbsolutePath("/home/work", "xx.txt"));
  EXPECT_EQ("/home/work/xx.txt", GetAbsolutePath("/home/work/", "xx.txt"));
  EXPECT_EQ("/xx.txt", GetAbsolutePath("/home/work", "/xx.txt"));
  EXPECT_EQ("/home/work/./xx.txt", GetAbsolutePath("/home/work", "./xx.txt"));
}

TEST_F(FileTest, GetFileNamesInFolderById) {
  std::string data_path = "modules/perception/data/hm_tracker_test/";
  std::vector<std::string> seg_filenames;
  GetFileNamesInFolderById(data_path, ".seg", &seg_filenames);
  std::vector<std::string> pose_filenames;
  GetFileNamesInFolderById(data_path, ".pose", &pose_filenames);
  EXPECT_EQ(seg_filenames.size(), 8);
  EXPECT_EQ(pose_filenames.size(), 8);
}

}  // namespace util
}  // namespace common
}  // namespace apollo
