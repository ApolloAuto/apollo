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

#ifndef MODULES_PERCEPTION_LIB_BASE_FILE_UTIL_H_
#define MODULES_PERCEPTION_LIB_BASE_FILE_UTIL_H_

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <string>
#include <vector>

#include "modules/common/macro.h"

namespace apollo {
namespace perception {

enum FileType { TYPE_FILE, TYPE_DIR };

// file name compared type
enum FileCompareType {
  FCT_DIGITAL = 0,
  FCT_LEXICOGRAPHICAL = 1,
  FCT_UNKNOWN = 8
};

class FileUtil {
 public:
  FileUtil() {}

  ~FileUtil() {}

  // check file or directory exists
  static bool Exists(const std::string& filename);

  // check whether file exists with [suffix] extension in [path]
  static bool Exists(const std::string& path, const std::string& suffix);

  // get file type: file/directory
  static bool GetType(const std::string& filename, FileType* type);

  // delete file or directory
  static bool DeleteFile(const std::string& filename);

  static bool CreateDir(const std::string& dir);

  static bool GetFileContent(const std::string& path, std::string* content);
  static bool ReadLines(const std::string& path,
                        std::vector<std::string>* lines);

  static std::string RemoveFileSuffix(std::string filename);

  static void GetFileList(const std::string& path, const std::string& suffix,
                          std::vector<std::string>* files);

  static void GetFileList(const std::string& path,
                          std::vector<std::string>* files);

  static std::string GetAbsolutePath(const std::string& prefix,
                                     const std::string& relative_path);

  // get file name
  // "/home/work/data/1.txt" -> 1
  static void GetFileName(const std::string& file, std::string* name);

  // return -1 when error occurred
  static int NumLines(const std::string& filename);

  // compare two file's name by digital value
  // "/home/work/data/1.txt" < "/home/user/data/10.txt"
  // "1.txt" < "./data/2.txt"
  static bool CompareFileByDigital(const std::string& file_left,
                                   const std::string& file_right);

  // compare two file's name by lexicographical order
  static bool CompareFileByLexicographical(const std::string& file_left,
                                           const std::string& file_right);

 private:
  static bool CompareFile(const std::string& file_left,
                          const std::string& file_right, FileCompareType type);

  DISALLOW_COPY_AND_ASSIGN(FileUtil);
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_LIB_BASE_FILE_UTIL_H_
