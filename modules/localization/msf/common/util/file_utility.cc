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

#include "modules/localization/msf/common/util/file_utility.h"

#include <dirent.h>

#include <algorithm>
#include <cerrno>
#include <iostream>
#include <limits>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include "cyber/common/log.h"
#include "fastrtps/TopicDataType.h"

namespace apollo {
namespace localization {
namespace msf {
const size_t kBufferSize = 20480000;

void FileUtility::ComputeFileMd5(const std::string &file_path,
                                 unsigned char res[kUcharMd5Length]) {
  std::vector<unsigned char> buf(kBufferSize);
  unsigned char *buf_pt = &buf[0];

  FILE *file = fopen(file_path.c_str(), "rb");
  size_t total_size = 0;
  if (file) {
    int count = 1;
    while (!feof(file)) {
      if (count > 1) {
        buf.resize(kBufferSize * count);
        buf_pt = &buf[count - 1];
      }

      size_t size = fread(buf_pt, sizeof(unsigned char), kBufferSize, file);
      total_size += size;

      ++count;
    }
  } else {
    AERROR << "Can't find the file: " << file_path;
    return;
  }

  ComputeBinaryMd5(&buf[0], total_size, res);
  fclose(file);
}

void FileUtility::ComputeFileMd5(const std::string &file_path,
                                 char res[kCharMd5Lenth]) {
  std::vector<unsigned char> buf(kBufferSize);
  unsigned char *buf_pt = &buf[0];

  FILE *file = fopen(file_path.c_str(), "rb");
  size_t total_size = 0;
  if (file) {
    int count = 1;
    while (!feof(file)) {
      if (count > 1) {
        buf.resize(kBufferSize * count);
        buf_pt = &buf[count - 1];
      }

      size_t size = fread(buf_pt, sizeof(unsigned char), kBufferSize, file);
      total_size += size;

      ++count;
    }
  } else {
    AERROR << "Can't find the file: " << file_path;
    return;
  }

  ComputeBinaryMd5(&buf[0], total_size, res);
  fclose(file);
}

void FileUtility::ComputeBinaryMd5(const unsigned char *binary, size_t size,
                                   unsigned char res[kUcharMd5Length]) {
  MD5 md5;
  md5.init();
  md5.update(binary, static_cast<unsigned int>(size));
  md5.finalize();
  for (uint8_t i = 0; i < kUcharMd5Length; ++i) {
    res[i] = md5.digest[i];
  }
}

void FileUtility::ComputeBinaryMd5(const unsigned char *binary, size_t size,
                                   char res[kCharMd5Lenth]) {
  unsigned char md[kUcharMd5Length] = {"\0"};
  char buf[kCharMd5Lenth] = {'\0'};
  char tmp[3] = {'\0'};

  ComputeBinaryMd5(binary, size, md);

  for (unsigned int i = 0; i < kUcharMd5Length; i++) {
    snprintf(tmp, sizeof(tmp), "%02X", md[i]);
    strncat(buf, tmp, sizeof(tmp));
  }

  memcpy(res, buf, sizeof(buf));
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
