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

#pragma once

#include <string>
#include <vector>

namespace apollo {
namespace localization {
namespace msf {

class system {
 public:
  /**@brief Determine if the file or directory exists. */
  static bool IsExists(const std::string& path);
  /**@brief Determine if the path is a directory. */
  static bool IsDirectory(const std::string& path);
  /**@brief Try to create a directory. */
  static bool CreateDirectory(const std::string& path);
  /**@brief Get the size of a file. */
  static bool GetFileSize(const std::string& path, unsigned int* size);
  /**@brief Copy the file. */
  static bool CopyFile(const std::string& src, const std::string& dst,
                       bool is_overwrite = true);
  /**@brief get list of files end with ext in folder.
   * @param <ext> should be .jpg instead of jpg. **/
  static void GetFilesInFolderRecursive(const std::string& folder,
                                        const std::string& ext,
                                        std::vector<std::string>* ret);
  /**@brief get list of files end with ext in folder.
   * @param <ext> should be .jpg instead of jpg. **/
  static void GetFilesInFolder(const std::string& folder,
                               const std::string& ext,
                               std::vector<std::string>* ret);
  /**@brief Get list of folders in folder. */
  static void GetFoldersInFolder(const std::string& folder,
                                 std::vector<std::string>* ret);
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
