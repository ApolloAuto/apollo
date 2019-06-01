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
#pragma once

#include <fcntl.h>
// #include <node/openssl/md5.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "cyber/common/log.h"

namespace apollo {
namespace localization {
namespace msf {

class FileUtility {
 public:
  static const size_t UCHAR_MD5LENTH = 16;
  static const size_t CHAR_MD5LENTH = 33;
  /**@brief Compute file md5 given a file path. */
  static void ComputeFileMd5(const std::string& file_path,
                             unsigned char res[UCHAR_MD5LENTH]);
  static void ComputeFileMd5(const std::string& file_path,
                             char res[CHAR_MD5LENTH]);
  /**@brief Compute file md5 given a binary chunk. */
  static void ComputeBinaryMd5(const unsigned char* binary, size_t size,
                               unsigned char res[UCHAR_MD5LENTH]);
  static void ComputeBinaryMd5(const unsigned char* binary, size_t size,
                               char res[CHAR_MD5LENTH]);
  /**
   * @brief Get file content as string.
   * @param file_name The name of the file to read content.
   * @param content The file content.
   * @return If the action is successful.
   */
  static bool GetContent(const std::string& file_name, std::string* content);

  /**
   * @brief Check if the path exists.
   * @return If the path exists.
   */
  static bool PathExists(const std::string& path);

  /**
   * @brief Check if the directory specified by directory_path exists
   *        and is indeed a directory.
   * @param directory_path Directory path.
   * @return If the directory specified by directory_path exists
   *         and is indeed a directory.
   */
  static bool DirectoryExists(const std::string& directory_path);

  /**
   * @brief Copy a file.
   * @param from The file path to copy from.
   * @param to The file path to copy to.
   * @return If the action is successful.
   */
  //  static bool CopyFile(const std::string& from, const std::string& to);

  /**
   * @brief Check if a specified directory specified by directory_path exists.
   *        If not, recursively create the directory (and its parents).
   * @param directory_path Directory path.
   * @return If the directory does exist or its creation is successful.
   */
  static bool EnsureDirectory(const std::string& directory_path);

  /**
   * @brief Remove all the files under a specified directory. Note that
   *        sub-directories are NOT affected.
   * @param directory_path Directory path.
   * @return If the action is successful.
   */
  static bool RemoveAllFiles(const std::string& directory_path);

  /**
   * @brief List sub-directories.
   * @param directory_path Directory path.
   * @return A vector of sub-directories, without the directory_path prefix.
   */
  static std::vector<std::string> ListSubDirectories(
      const std::string& directory_path);

  /**@brief Determine if the file or directory exists. */
  static bool IsExists(const std::string& path);
  /**@brief Determine if the path is a directory. */
  static bool IsDirectory(const std::string& path);
  /**@brief Try to create a directory. */
  static bool CreateDirectory(const std::string& path);
  /**@brief Get the size of a file. */
  static bool GetFileSize(const std::string& path, size_t* size);
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
