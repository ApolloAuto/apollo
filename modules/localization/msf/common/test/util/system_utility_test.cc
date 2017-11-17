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
#include <gtest/gtest.h>
#include <boost/filesystem.hpp>

namespace apollo {
namespace localization {
namespace msf {

class SystemUtilityTestSuite : public ::testing::Test {
 protected:
  SystemUtilityTestSuite() {}
  virtual ~SystemUtilityTestSuite() {}
  virtual void SetUp() {}
  virtual void TearDown() {}
};

/**@brief Rect2DTest. */
TEST_F(SystemUtilityTestSuite, SystemTest) {
  bool flag =
      system::IsExists("modules/localization/msf/common/util/system_utility.h");
  ASSERT_TRUE(flag);
  flag = system::IsExists(
      "modules/localization/msf/common/util/system_utility.hh");
  ASSERT_FALSE(flag);
  flag = system::IsDirectory("modules/localization/msf/common/util");
  ASSERT_TRUE(flag);
  flag = system::IsDirectory(
      "modules/localization/msf/common/util/system_utility.h");
  ASSERT_FALSE(flag);
  flag = system::CreateDirectory(
      "modules/localization/msf/common/test/test_data/tem");
  ASSERT_TRUE(flag);
  unsigned int size;
  flag = system::GetFileSize(
      "modules/localization/msf/common/test/test_data/test_folder/file1.txt",
      size);
  ASSERT_TRUE(flag);
  ASSERT_EQ(size, 1);
  flag = system::CopyFile(
      "modules/localization/msf/common/test/test_data/test_folder/file1.txt",
      "modules/localization/msf/common/test/test_data/tem/file1.txt");
  ASSERT_TRUE(flag);
  std::vector<std::string> ret1;
  system::GetFilesInFolderRecursive(
      "modules/localization/msf/common/test/test_data", ".txt", ret1);
  ASSERT_EQ(ret1.size(), 4);
  std::vector<std::string> ret2;
  system::GetFilesInFolder(
      "modules/localization/msf/common/test/test_data/test_folder", ".txt",
      ret2);
  ASSERT_EQ(ret2.size(), 3);
  std::vector<std::string> ret3;
  system::GetFoldersInFolder("modules/localization/msf/common/test/test_data/",
                             ret3);
  ASSERT_EQ(ret3.size(), 2);
  boost::filesystem::remove_all(
      "modules/localization/msf/common/test/test_data/tem");
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
