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

#include "modules/localization/msf/common/util/file_utility.h"

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>

namespace apollo {
namespace localization {
namespace msf {

class FileUtilityTestSuite : public ::testing::Test {
 protected:
  FileUtilityTestSuite() {}
  virtual ~FileUtilityTestSuite() {}
  virtual void SetUp() {}
  virtual void TearDown() {}
};

/**@brief Rect2DTest. */
TEST_F(FileUtilityTestSuite, SystemTest) {
  bool flag = FileUtility::IsExists(
      "/apollo/modules/localization/msf/common/test_data/test_folder/"
      "file1.txt");
  ASSERT_TRUE(flag);
  flag = FileUtility::IsExists(
      "/apollo/modules/localization/msf/common/test_data/file4.txt");
  ASSERT_FALSE(flag);
  flag =
      FileUtility::IsDirectory("/apollo/modules/localization/msf/common/test_data");
  ASSERT_TRUE(flag);
  flag = FileUtility::IsDirectory("/apollo/modules/localization/msf/common/test");
  ASSERT_FALSE(flag);
  flag = FileUtility::CreateDirectory(
      "/apollo/modules/localization/msf/common/test_data/tem");
  ASSERT_TRUE(flag);
  size_t size;
  flag = FileUtility::GetFileSize(
      "/apollo/modules/localization/msf/common/test_data/test_folder/file1.txt",
      &size);
  ASSERT_TRUE(flag);
  ASSERT_EQ(size, 1);
  flag = FileUtility::CopyFile(
      "/apollo/modules/localization/msf/common/test_data/test_folder/file1.txt",
      "/apollo/modules/localization/msf/common/test_data/tem/file1.txt");
  ASSERT_TRUE(flag);
  std::vector<std::string> ret1;
  FileUtility::GetFilesInFolderRecursive(
      "/apollo/modules/localization/msf/common/test_data", ".txt", &ret1);
  ASSERT_EQ(ret1.size(), 4);
  std::vector<std::string> ret2;
  FileUtility::GetFilesInFolder(
      "/apollo/modules/localization/msf/common/test_data/test_folder", ".txt",
      &ret2);
  ASSERT_EQ(ret2.size(), 3);
  std::vector<std::string> ret3;
  FileUtility::GetFoldersInFolder(
      "/apollo/modules/localization/msf/common/test_data/", &ret3);
  ASSERT_EQ(ret3.size(), 2);
  boost::filesystem::remove_all(
      "/apollo/modules/localization/msf/common/test_data/tem");
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
