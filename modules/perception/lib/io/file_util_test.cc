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

// TODO(all): to add back
// TEST(FileUtilTest, TestGetType) {
//   FileType type;
//   EXPECT_FALSE(FileUtil::GetType("../testdata/data/2.txt", &type));
//   EXPECT_FALSE(FileUtil::GetType("../testdata/data/-/*/", &type));
//   EXPECT_TRUE(FileUtil::GetType("../testdata/data", &type));
// }

TEST(FileUtilTest, TestCreateDir) {
  EXPECT_TRUE(FileUtil::CreateDir("../testdata/data3/data4/data5"));
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
  std::string name;
  FileUtil::GetFileName("/home/work/data/1.txt", &name);
  EXPECT_EQ("1", name);
  FileUtil::GetFileName("../testdata/data/data/11.txt", &name);
  EXPECT_EQ("11", name);
  FileUtil::GetFileName("111.txt", &name);
  EXPECT_EQ("111", name);
  FileUtil::GetFileName("111", &name);
  EXPECT_EQ("111", name);
}

TEST(FileUtilTest, TestCompareFileByDigital) {
  EXPECT_TRUE(FileUtil::CompareFileByDigital("../testdata/data/1.txt",
                                             "/home/data/2.txt"));
  EXPECT_TRUE(FileUtil::CompareFileByDigital("1.txt", "/home/data/2.txt"));
  EXPECT_TRUE(FileUtil::CompareFileByDigital("01.txt", "/home/data/2.txt"));
  EXPECT_FALSE(FileUtil::CompareFileByDigital("10", "/home/data/2.txt"));
  EXPECT_FALSE(FileUtil::CompareFileByDigital("010", "/home/data/2.txt"));
  EXPECT_FALSE(FileUtil::CompareFileByDigital("100", "001"));
}

TEST(FileUtilTest, TestCompareFileByLexicographical) {
  EXPECT_TRUE(FileUtil::CompareFileByLexicographical(
      "../testdata/data/QB1234_2222_3333_0001.pcd",
      "../testdata/data/QB1234_2222_3333_1000.pcd"));
  EXPECT_TRUE(FileUtil::CompareFileByLexicographical(
      "../testdata/data/QB1234_2222_3333_0001.pcd",
      "../testdata/data/QB1234_3333_3333_1000.pcd"));
}

// TODO(all): to add back
// TEST(FileUtilTest, TestCompareFile) {
//   EXPECT_TRUE(FileUtil::CompareFile("1.txt", "1.txt", FCT_UNKNOWN));
// }

/* TODO(all): to add back
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
  ASSERT_EQ(system("chmod -R -x ../testdata/data3"), 256);
  ASSERT_FALSE(FileUtil::DeleteFile("../testdata/data3"));
  ASSERT_EQ(system("chmod -R +x ../testdata/data3"), 0);
  ASSERT_EQ(system("chmod -R -r ../testdata/data3"), 256);
  ASSERT_FALSE(FileUtil::DeleteFile("../testdata/data3"));
  ASSERT_EQ(system("chmod -R +r ../testdata/data3"), 0);
  ASSERT_EQ(system("chmod -R -w ../testdata/data3"), 0);
  ASSERT_FALSE(FileUtil::DeleteFile("../testdata/data3"));
  ASSERT_EQ(system("chmod -R +w ../testdata/data3"), 0);
  ASSERT_EQ(system("mkdir -p tmpa/tmpb"), 0);
  ASSERT_EQ(system("mkdir -p tmpa/tmpc"), 0);
  ASSERT_EQ(system("touch tmpa/tmpb/b.txt"), 0);
  ASSERT_EQ(system("touch tmpa/tmpb/b2.txt"), 0);
  ASSERT_EQ(system("touch tmpa/a.txt"), 0);
  ASSERT_TRUE(FileUtil::DeleteFile("./tmpa/tmpb/b.txt"));
  ASSERT_TRUE(FileUtil::DeleteFile("./tmpa/tmpb"));
  ASSERT_TRUE(FileUtil::DeleteFile("./tmpa"));
}
*/

/* TODO(all): to add back
TEST(FileUtilTest, TestReadLines) {
  std::string path = "../testdata/data/1.txt";
  std::vector<std::string> lines;
  EXPECT_FALSE(FileUtil::ReadLines("/not_exist_path", &lines));
  EXPECT_FALSE(FileUtil::ReadLines("../testdata/data/1.txt", nullptr));
  EXPECT_TRUE(FileUtil::ReadLines(path, &lines));
  EXPECT_EQ(lines.size(), 2u);
}
*/

TEST(FileUtilTest, TestRemoveFileSuffix) {
  EXPECT_EQ(FileUtil::RemoveFileSuffix("../testdata/data/1.txt"), "1");
  EXPECT_EQ(FileUtil::RemoveFileSuffix("/home/111"), "111");
}

/* TODO(all): to add back
TEST(FileUtilTest, TestGetFileList) {
  std::string path = "../testdata/data";
  std::vector<std::string> files;
  EXPECT_TRUE(FileUtil::GetFileList(path, &files));
  EXPECT_FALSE(FileUtil::GetFileList("/not_exist_path", &files));
  EXPECT_TRUE(FileUtil::GetFileList("../testdata/data", "txt", &files));
}
*/

/* TODO(all): to add back
TEST(FileUtilTest, TestNumLines) {
  EXPECT_EQ(FileUtil::NumLines("../testdata/data/1.txt"), 3);
  EXPECT_EQ(FileUtil::NumLines("../testdata/data/11.txt"), -1);
}
*/

/* TODO(all): to add back
TEST(FileUtilTest, TestRenameFile) {
  EXPECT_TRUE(FileUtil::RenameFile("../testdata/data2/123.txt",
                                   "../testdata/data2/321.txt"));
  EXPECT_TRUE(FileUtil::RenameFile("../testdata/data2/321.txt",
                                   "../testdata/data2/123.txt"));
  EXPECT_FALSE(FileUtil::RenameFile("../testdata/data2/111.txt",
                                    "../testdata/data2/222.txt"));
}
*/

/* TODO(all): to add back
TEST(FileUtilTest, TestGetFileContent) {
  std::string path = "../testdata/data/1.txt";
  std::string content;
  EXPECT_FALSE(FileUtil::GetFileContent(path, NULL));
  EXPECT_FALSE(
      FileUtil::GetFileContent("../testdata/data/2.txt", &content));
  EXPECT_TRUE(FileUtil::GetFileContent(path, &content));
  ASSERT_EQ(system("chmod a-r ../testdata/data3/1.txt"), 0);
  ASSERT_FALSE(
      FileUtil::GetFileContent("../testdata/data3/1.txt", &content));
  ASSERT_EQ(system("chmod a+r ../testdata/data3/1.txt"), 0);
}
*/

TEST(FileUtilTest, TestFileUtil) { FileUtil file_util; }

}  // namespace lib
}  // namespace perception
}  // namespace apollo
