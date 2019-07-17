/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
