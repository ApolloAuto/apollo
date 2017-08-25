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

#include "modules/perception/lib/base/file_util.h"

#include <fstream>
#include <string>
#include <vector>
#include "gtest/gtest.h"

namespace apollo {
namespace perception {

using std::string;

TEST(FileUtilTest, test_GetAbsolutePath) {
  EXPECT_EQ(FileUtil::GetAbsolutePath("", "./xx.txt"), "./xx.txt");
  EXPECT_EQ(FileUtil::GetAbsolutePath("/abc", ""), "/abc");
  EXPECT_EQ(FileUtil::GetAbsolutePath("/home/work", "xx.txt"),
            "/home/work/xx.txt");
  EXPECT_EQ(FileUtil::GetAbsolutePath("/home/work/", "xx.txt"),
            "/home/work/xx.txt");
  EXPECT_EQ(FileUtil::GetAbsolutePath("/home/work", "/xx.txt"), "/xx.txt");
  EXPECT_EQ(FileUtil::GetAbsolutePath("/home/work", "./xx.txt"),
            "/home/work/./xx.txt");
}

TEST(FileUtilTest, test_GetFileName) {
  string name;
  FileUtil::GetFileName("/home/work/data/1.txt", &name);
  EXPECT_EQ("1", name);
  FileUtil::GetFileName("./data/data/11.txt", &name);
  EXPECT_EQ("11", name);
  FileUtil::GetFileName("111.txt", &name);
  EXPECT_EQ("111", name);
  FileUtil::GetFileName("111", &name);
  EXPECT_EQ("111", name);
}

TEST(FileUtilTest, test_CompareFileByDigital) {
  EXPECT_TRUE(
      FileUtil::CompareFileByDigital("./data/1.txt", "/home/data/2.txt"));
  EXPECT_TRUE(FileUtil::CompareFileByDigital("1.txt", "/home/data/2.txt"));
  EXPECT_TRUE(FileUtil::CompareFileByDigital("01.txt", "/home/data/2.txt"));
  EXPECT_FALSE(FileUtil::CompareFileByDigital("10", "/home/data/2.txt"));
  EXPECT_FALSE(FileUtil::CompareFileByDigital("010", "/home/data/2.txt"));
  EXPECT_FALSE(FileUtil::CompareFileByDigital("100", "001"));
}

TEST(FileUtilTest, test_CompareFileByLexicographical) {
  EXPECT_TRUE(FileUtil::CompareFileByLexicographical(
      "./data/QB1234_2222_3333_0001.pcd", "./data/QB1234_2222_3333_1000.pcd"));
  EXPECT_TRUE(FileUtil::CompareFileByLexicographical(
      "./data/QB1234_2222_3333_0001.pcd", "./data/QB1234_3333_3333_1000.pcd"));
}

TEST(FileUtilTest, test_exists) {
  ASSERT_TRUE(FileUtil::CreateDir("./tmp"));
  EXPECT_FALSE(FileUtil::Exists("./tmp", ".txt"));
  ASSERT_EQ(std::system("touch ./tmp/a.txt"), 0);
  EXPECT_TRUE(FileUtil::Exists("./tmp", ".txt"));

  FileType type;
  ASSERT_TRUE(FileUtil::GetType("./tmp", &type));
  ASSERT_TRUE(FileUtil::GetType("./tmp/a.txt", &type));
  std::system("ln -s ./tmp/a.txt ./ln_a.txt");
  ASSERT_FALSE(FileUtil::GetType("./ln_a.txt", &type));

  ASSERT_TRUE(FileUtil::DeleteFile("/not_exist_path"));
  ASSERT_TRUE(FileUtil::DeleteFile("./tmp"));
}

TEST(FileUtilTest, test_ReadLines) {
  std::string data_file = "./1.txt";
  std::system("echo -e '111\n222' > 1.txt");

  std::vector<std::string> lines;
  EXPECT_FALSE(FileUtil::ReadLines("/not_exist_path", &lines));
  EXPECT_FALSE(FileUtil::ReadLines("./1.txt", nullptr));

  EXPECT_TRUE(FileUtil::ReadLines(data_file, &lines));
  EXPECT_EQ(lines.size(), 2u);
  std::system("rm ./1.txt");
}

TEST(FileUtilTest, test_GetFileContent) {
  std::string proto_file =
      "modules/perception/data/config_manager_test/config_manager.config";
  std::string content;
  EXPECT_FALSE(FileUtil::GetFileContent(proto_file, NULL));
  EXPECT_FALSE(FileUtil::GetFileContent("/not_exist_path", &content));
  EXPECT_TRUE(FileUtil::GetFileContent(proto_file, &content));
}

TEST(FileUtilTest, test_RemoveFileSuffix) {
  std::string proto_file =
      "modules/perception/data/config_manager_test/config_manager.config";
  EXPECT_EQ("config_manager", FileUtil::RemoveFileSuffix(proto_file));

  proto_file = "config_manager";
  EXPECT_EQ("config_manager", FileUtil::RemoveFileSuffix(proto_file));
}

TEST(FileUtilTest, test_GetFileList) {
  std::vector<std::string> files;
  FileUtil::GetFileList("/not_exist_path", "config", &files);
  EXPECT_EQ(0, files.size());
  std::string path = "modules/perception/data/config_manager_test";
  FileUtil::GetFileList(path, "config", &files);
  EXPECT_GT(files.size(), 0);

  files.clear();
  FileUtil::GetFileList(path, &files);
  EXPECT_GT(files.size(), 0);
}

TEST(FileUtilTest, test_NumLines) {
  std::string proto_file =
      "modules/perception/data/config_manager_test/config_manager.config";
  EXPECT_GT(FileUtil::NumLines(proto_file), 0);
}

}  // namespace perception
}  // namespace apollo
