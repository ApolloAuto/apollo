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
#include "modules/perception/lib/io/file_util.h"

#include <gtest/gtest.h>

namespace apollo {
namespace perception {
namespace lib {

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
  FileUtil::GetFileName(
      "/apollo/modules/perception/testdata/lib/data/data/11.txt", &name);
  EXPECT_EQ("11", name);
  FileUtil::GetFileName("111.txt", &name);
  EXPECT_EQ("111", name);
  FileUtil::GetFileName("111", &name);
  EXPECT_EQ("111", name);
}

TEST(FileUtilTest, TestGetFileList) {
  std::string path = "/apollo/modules/perception/testdata/lib/data";
  std::vector<std::string> files;
  EXPECT_TRUE(FileUtil::GetFileList(path, "", &files));
  EXPECT_FALSE(FileUtil::GetFileList("/not_exist_path", "", &files));
  EXPECT_TRUE(FileUtil::GetFileList(
      "/apollo/modules/perception/testdata/lib/data", "txt", &files));
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo
