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

#include "modules/perception/obstacle/common/file_system_util.h"

#include "gtest/gtest.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {

TEST(FileSystemUtilTest, GetFileNamesInFolderById) {
  std::string data_path = "modules/perception/data/hm_tracker_test/";
  std::vector<std::string> seg_filenames;
  GetFileNamesInFolderById(data_path, ".seg", &seg_filenames);
  std::vector<std::string> pose_filenames;
  GetFileNamesInFolderById(data_path, ".pose", &pose_filenames);
  EXPECT_EQ(seg_filenames.size(), 8);
  EXPECT_EQ(pose_filenames.size(), 8);
}

}  // namespace perception
}  // namespace apollo
