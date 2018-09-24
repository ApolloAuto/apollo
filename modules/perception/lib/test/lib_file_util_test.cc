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

#include "modules/perception/lib/io/file_util.h"

namespace apollo {
namespace perception {
namespace lib {

using std::string;
using std::vector;

TEST(FileUtilTest, TestGetType) {
  FileType type;
  EXPECT_FALSE(FileUtil::GetType("./lib_test_data/data/2.txt", &type));
  EXPECT_FALSE(FileUtil::GetType("./lib_test_data/data/-/*/", &type));
  EXPECT_TRUE(FileUtil::GetType("./lib_test_data/data", &type));
}

TEST(FileUtilTest, TestCreateDir) {
  EXPECT_TRUE(FileUtil::CreateDir("./lib_test_data/data3/data4/data5"));
}

TEST(FileUtilTest, TestGetAbsolutPath) {
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

TEST(FileUtilTest, TestGetFileName) {
  string name;
  FileUtil::GetFileName("/home/work/data/1.txt", &name);
  EXPECT_EQ("1", name);
  FileUtil::GetFileName("./lib_test_data/data/data/11.txt", &name);
  EXPECT_EQ("11", name);
  FileUtil::GetFileName("111.txt", &name);
  EXPECT_EQ("111", name);
  FileUtil::GetFileName("111", &name);
  EXPECT_EQ("111", name);
}

TEST(FileUtilTest, TestCompareFileByDigital) {
  EXPECT_TRUE(FileUtil::CompareFileByDigital("./lib_test_data/data/1.txt",
                                             "/home/data/2.txt"));
  EXPECT_TRUE(FileUtil::CompareFileByDigital("1.txt", "/home/data/2.txt"));
  EXPECT_TRUE(FileUtil::CompareFileByDigital("01.txt", "/home/data/2.txt"));
  EXPECT_FALSE(FileUtil::CompareFileByDigital("10", "/home/data/2.txt"));
  EXPECT_FALSE(FileUtil::CompareFileByDigital("010", "/home/data/2.txt"));
  EXPECT_FALSE(FileUtil::CompareFileByDigital("100", "001"));
}

TEST(FileUtilTest, TestCompareFileByLexicographical) {
  EXPECT_TRUE(FileUtil::CompareFileByLexicographical(
      "./lib_test_data/data/QB1234_2222_3333_0001.pcd",
      "./lib_test_data/data/QB1234_2222_3333_1000.pcd"));
  EXPECT_TRUE(FileUtil::CompareFileByLexicographical(
      "./lib_test_data/data/QB1234_2222_3333_0001.pcd",
      "./lib_test_data/data/QB1234_3333_3333_1000.pcd"));
}

TEST(FileUtilTest, TestCompareFile) {
  EXPECT_TRUE(FileUtil::CompareFile("1.txt", "1.txt", FCT_UNKNOWN));
}

TEST(FileUtilTest, TestExists) {
  ASSERT_TRUE(FileUtil::CreateDir("./tmp"));
  EXPECT_TRUE(FileUtil::CreateDir(""));
  EXPECT_FALSE(FileUtil::Exists("./tmp", ".txt"));
  ASSERT_EQ(system("touch ./tmp/a.txt"), 0);
  EXPECT_TRUE(FileUtil::Exists("./tmp", ".txt"));
  EXPECT_FALSE(FileUtil::Exists("./tmp", ".bat"));
  EXPECT_TRUE(FileUtil::Exists("./tmp/a.txt"));
  EXPECT_FALSE(FileUtil::Exists("./tmp1"));
  ASSERT_TRUE(FileUtil::DeleteFile("./tmp/a.txt"));
  ASSERT_TRUE(FileUtil::DeleteFile("./tmp"));
  ASSERT_TRUE(FileUtil::DeleteFile("./tmp1"));
  ASSERT_EQ(system("chmod -R -x ./lib_test_data/data3"), 256);
  ASSERT_FALSE(FileUtil::DeleteFile("./lib_test_data/data3"));
  ASSERT_EQ(system("chmod -R +x ./lib_test_data/data3"), 0);
  ASSERT_EQ(system("chmod -R -r ./lib_test_data/data3"), 256);
  ASSERT_FALSE(FileUtil::DeleteFile("./lib_test_data/data3"));
  ASSERT_EQ(system("chmod -R +r ./lib_test_data/data3"), 0);
  ASSERT_EQ(system("chmod -R -w ./lib_test_data/data3"), 0);
  ASSERT_FALSE(FileUtil::DeleteFile("./lib_test_data/data3"));
  ASSERT_EQ(system("chmod -R +w ./lib_test_data/data3"), 0);
  ASSERT_EQ(system("mkdir -p tmpa/tmpb"), 0);
  ASSERT_EQ(system("mkdir -p tmpa/tmpc"), 0);
  ASSERT_EQ(system("touch tmpa/tmpb/b.txt"), 0);
  ASSERT_EQ(system("touch tmpa/tmpb/b2.txt"), 0);
  ASSERT_EQ(system("touch tmpa/a.txt"), 0);
  ASSERT_TRUE(FileUtil::DeleteFile("./tmpa/tmpb/b.txt"));
  ASSERT_TRUE(FileUtil::DeleteFile("./tmpa/tmpb"));
  ASSERT_TRUE(FileUtil::DeleteFile("./tmpa"));
}

TEST(FileUtilTest, TestReadLines) {
  string path = "./lib_test_data/data/1.txt";
  vector<string> lines;
  EXPECT_FALSE(FileUtil::ReadLines("/not_exist_path", &lines));
  EXPECT_FALSE(FileUtil::ReadLines("./lib_test_data/data/1.txt", nullptr));
  EXPECT_TRUE(FileUtil::ReadLines(path, &lines));
  EXPECT_EQ(lines.size(), 2u);
}

TEST(FileUtilTest, TestRemoveFileSuffix) {
  EXPECT_EQ(FileUtil::RemoveFileSuffix("./lib_test_data/data/1.txt"), "1");
  EXPECT_EQ(FileUtil::RemoveFileSuffix("/home/111"), "111");
}

TEST(FileUtilTest, TestGetFileList) {
  string path = "./lib_test_data/data";
  vector<string> files;
  EXPECT_TRUE(FileUtil::GetFileList(path, &files));
  EXPECT_FALSE(FileUtil::GetFileList("/not_exist_path", &files));
  EXPECT_TRUE(FileUtil::GetFileList("./lib_test_data/data", "txt", &files));
}

TEST(FileUtilTest, TestNumLines) {
  EXPECT_EQ(FileUtil::NumLines("./lib_test_data/data/1.txt"), 3);
  EXPECT_EQ(FileUtil::NumLines("./lib_test_data/data/11.txt"), -1);
}

TEST(FileUtilTest, TestRenameFile) {
  EXPECT_TRUE(FileUtil::RenameFile("./lib_test_data/data2/123.txt",
                                   "./lib_test_data/data2/321.txt"));
  EXPECT_TRUE(FileUtil::RenameFile("./lib_test_data/data2/321.txt",
                                   "./lib_test_data/data2/123.txt"));
  EXPECT_FALSE(FileUtil::RenameFile("./lib_test_data/data2/111.txt",
                                    "./lib_test_data/data2/222.txt"));
}

TEST(FileUtilTest, TestGetFileContent) {
  string path = "./lib_test_data/data/1.txt";
  string content;
  EXPECT_FALSE(FileUtil::GetFileContent(path, NULL));
  EXPECT_FALSE(
      FileUtil::GetFileContent("./lib_test_data/data/2.txt", &content));
  EXPECT_TRUE(FileUtil::GetFileContent(path, &content));
  ASSERT_EQ(system("chmod a-r ./lib_test_data/data3/1.txt"), 0);
  ASSERT_FALSE(
      FileUtil::GetFileContent("./lib_test_data/data3/1.txt", &content));
  ASSERT_EQ(system("chmod a+r ./lib_test_data/data3/1.txt"), 0);
}

TEST(FileUtilTest, TestFileUtil) { FileUtil file_util; }

}  // namespace lib
}  // namespace perception
}  // namespace apollo
