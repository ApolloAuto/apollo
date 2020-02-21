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

#include "modules/localization/msf/common/util/system_utility.h"

#include <boost/filesystem.hpp>

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace msf {

TEST(SystemUtilityTestSuite, SystemTest) {
  bool flag = system::IsExists(
      "/apollo/modules/localization/msf/common/test_data/test_folder/"
      "file1.txt");
  EXPECT_TRUE(flag);
  flag = system::IsExists(
      "/apollo/modules/localization/msf/common/test_data/file4.txt");
  EXPECT_FALSE(flag);
  flag =
      system::IsDirectory("/apollo/modules/localization/msf/common/test_data");
  EXPECT_TRUE(flag);
  flag = system::IsDirectory("/apollo/modules/localization/msf/common/test");
  EXPECT_FALSE(flag);
  flag = system::CreateDirectory(
      "/apollo/modules/localization/msf/common/test_data/tem");
  EXPECT_TRUE(flag);
  unsigned int size;
  flag = system::GetFileSize(
      "/apollo/modules/localization/msf/common/test_data/test_folder/file1.txt",
      &size);
  EXPECT_TRUE(flag);
  EXPECT_EQ(size, 1);
  flag = system::CopyFile(
      "/apollo/modules/localization/msf/common/test_data/test_folder/file1.txt",
      "/apollo/modules/localization/msf/common/test_data/tem/file1.txt");
  EXPECT_TRUE(flag);
  std::vector<std::string> ret1;
  system::GetFilesInFolderRecursive(
      "/apollo/modules/localization/msf/common/test_data", ".txt", &ret1);
  EXPECT_EQ(ret1.size(), 4);
  std::vector<std::string> ret2;
  system::GetFilesInFolder(
      "/apollo/modules/localization/msf/common/test_data/test_folder", ".txt",
      &ret2);
  EXPECT_EQ(ret2.size(), 3);
  std::vector<std::string> ret3;
  system::GetFoldersInFolder(
      "/apollo/modules/localization/msf/common/test_data/", &ret3);
  EXPECT_EQ(ret3.size(), 2);
  boost::filesystem::remove_all(
      "/apollo/modules/localization/msf/common/test_data/tem");
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
